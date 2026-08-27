#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""带载老化 (圆柱抓握耐久) —— 对齐上位机 `SweepWorker::doStartGrasp`。

**动作**: 整手在「张开」↔「抓握圆柱」两个姿态之间**同相**正弦开合, 0.25 Hz
(一次开合 4 秒), 无限循环。拇指参与对掌。

    pos = center + amp · env · sin(ωt)        center=(张开+抓握)/2
    vel = amp · env · ω · cos(ωt)             amp  =(抓握-张开)/2   ← 有符号

`amp` 保留符号是关键: 所有轴在 sin=+1 时**同时**到达抓握位, 不会因为某些轴的
目标是负角而反相。相位全部为 0 —— 带载场景手里握着圆柱, 靠工件本身撑开手指,
不需要空载老化那套反相防撞。

**抓握姿态**从真机手摆采集 (2026-07-07, 右手, 只读采集), 再按两个百分比缩放:
  --depth   抓握深度%  (默认 60): 缩放名义抓握姿态。粗圆柱 → 浅, 细 → 深。
  --release 张开程度%  (默认 30): 张开位 = 抓握位 × 该比例。值越小张得越开,
            0 = 完全伸直, 100 = 一直握着不松。

行程口径见下方 `LIMIT_RELATIVE` 常量的注释。
"""

from __future__ import annotations

import math
import sys
from typing import List

from aging_common import (
    D2R, PI, R2D, TOTAL_JOINTS,
    HandRunner, JointLimits, Profile,
    build_argparser, finger_of, run_multi, slot_of,
)

# ── 运动参数 (对应上位机「运动参数中心 → 带载老化」一行) ─────────────────
FREQ_HZ = 0.25          # MotionParamStore::graspFreqHz 默认值
# 四指屈曲轴的闭合口径: False = 用下面采集的名义角(默认, 保守);
# True = 按各关节限位比例闭合(上位机读得到设备真限位时走这条)。demo 只有设计限位,
# 名义角更稳妥, 所以写死 False; 换成实测限位表后可以改成 True 试。
LIMIT_RELATIVE = False
KP, KD, IQ = 5.0, 0.1, 1.5      # DEFS[GraspAging] = {5.0, 0.10, 1.5}
RAMP_FRACTION = 0.25
MARGIN_RAD = 3.0 * D2R  # 夹关节限位余量

# depth=100% 时的满深抓握名义姿态(度)。行 = 手指 (0拇指 1食 2中 3无名 4小),
# 列 = 轴 (0 flex/cmc_flex, 1 abd, 2 pip/mcp, 3 dip/ip)。掌心方向为 +;
# 拇指 cmc_abd 负值 = 对掌。2026-07-07 右手手摆采集。
GRASP_POSE_DEG = (
    (+62.0, -92.0, +90.0, +108.0),   # 拇指
    (+75.0,  -7.4, +130.0, +100.0),  # 食指
    (+90.0,  +4.6, +130.0, +100.0),  # 中指
    (+84.0, +14.2, +130.0, +100.0),  # 无名指
    (+73.0, +33.1, +130.0, +100.0),  # 小指
)

# 四指外展摆动 (与空载老化共用同一套系数, 改一处两处都要改)
ABD_FLAT_IDX = (5, 9, 13, 17)
ABD_SWING_SCALE = (1.0, 1.0, 0.6, 0.5)
FAN_SPREAD_SIGN = (-1.0, -1.0, +1.0, +1.0)
FAN_SPREAD_FRAC = (1.0, 0.333, 0.333, 1.0)
FAN_MARGIN_RAD = 5.0 * D2R


def build_profile(ctx: HandRunner) -> Profile:
    limits: JointLimits = ctx.limits
    args = ctx.args
    depth = args.depth / 100.0
    release_frac = args.release / 100.0

    center = [0.0] * TOTAL_JOINTS
    amp = [0.0] * TOTAL_JOINTS
    enabled = [False] * TOTAL_JOINTS

    for k in range(TOTAL_JOINTS):
        rng = limits.get(k)
        if rng is None:
            continue
        enabled[k] = True
        bus, slot = finger_of(k), slot_of(k)
        finger_flex = (bus != 0) and slot in (0, 2, 3)

        if LIMIT_RELATIVE and finger_flex:
            # 限位相对: 闭合方向朝 max(屈曲)。从中性 0 按 depth 缩放 —— 锚在
            # min_rad(伸展端)会导致 depth<100% 时远远握不拢。
            close_lim = rng[1] - MARGIN_RAD
            grasp = min(depth * close_lim, close_lim)
            opened = release_frac * grasp
        else:
            grasp_rad = GRASP_POSE_DEG[bus][slot] * D2R * depth
            open_rad = release_frac * grasp_rad
            if bus == 0 and slot == 1:
                open_rad = -40.0 * D2R * depth   # 拇指 cmc_abd 固定张开位
            hi = rng[1] - MARGIN_RAD
            lo = rng[0] + MARGIN_RAD
            if bus == 0:                          # 拇指: 放宽到机械可达
                if slot == 1:
                    lo = -95.0 * D2R
                if slot == 2:
                    hi = +92.0 * D2R
                if slot == 3:
                    hi = +110.0 * D2R
            else:                                 # 四指
                if slot == 0:
                    hi = rng[1] - 1.0 * D2R
                if slot == 2:
                    hi = +132.0 * D2R
                if slot == 3:
                    hi = +102.0 * D2R
            grasp = min(hi, max(lo, grasp_rad))
            opened = min(hi, max(lo, open_rad))

        center[k] = (opened + grasp) * 0.5
        amp[k] = (grasp - opened) * 0.5           # 有符号 → 全轴同相到位

    # 四指外展摆动: 开启时**覆盖**抓握姿态表给这四个轴算出的 center/amp
    # (对齐上位机 doStartGrasp —— 摆动开关 ON 时 abd 不再跟着抓握开合走)。
    # 中心由 --abd-mode 决定: manual=锁定当前手动撑开位 / fan=按设备限位张最大。
    # 带载没有物理负载压着四指, 所以四指可以自由张开摆动。
    notes: List[str] = []
    if args.abd_swing_deg > 0.0:
        swing_rad = args.abd_swing_deg * D2R
        for i, k in enumerate(ABD_FLAT_IDX):
            if args.abd_mode == "manual":
                center[k] = ctx.actual[k]
            else:                                   # fan: 按本手限位张最大扇形
                rng = limits.get(k)
                if rng is None:
                    continue
                mid = (rng[0] + rng[1]) * 0.5
                reach = max(0.0, (rng[1] - rng[0]) * 0.5 - FAN_MARGIN_RAD)
                center[k] = mid + FAN_SPREAD_SIGN[i] * FAN_SPREAD_FRAC[i] * reach
            amp[k] = swing_rad * ABD_SWING_SCALE[i]
            # 同空载老化: 摆幅按可用余量夹住, 免得每周期顶在限位上白摆还发热。
            rng = limits.get(k)
            if rng is not None:
                room = max(0.0, min(center[k] - rng[0], rng[1] - center[k]) - FAN_MARGIN_RAD)
                amp[k] = min(amp[k], room)
        notes.append("四指外展: 中心=%s [%s]° + 绕中心 ±%.1f° 逐指分级摆动 (覆盖抓握 abd)"
                     % ("锁定当前手动撑开位" if args.abd_mode == "manual" else "按设备限位张最大",
                        ", ".join("%.0f" % (center[k] * R2D) for k in ABD_FLAT_IDX),
                        args.abd_swing_deg))
    else:
        notes.append("四指外展: 跟随抓握姿态开合 (--abd-swing-deg 0 = 二轴不单独摆)")

    notes.append("深度 %d%% · 张开 %d%% · %.2fHz · %s口径"
                 % (args.depth, args.release, FREQ_HZ,
                    "限位相对" if LIMIT_RELATIVE else "名义角"))
    notes.append("抓握姿态表是**右手**采集的; 左手若外展方向不对需自行镜像 abd 轴。"
                 if str(ctx.handedness).lower().startswith("l") else
                 "抓握姿态表来自右手采集, 与当前手型一致。")

    omega = 2.0 * PI * FREQ_HZ

    def motion(t: float, env: float):
        s = math.sin(omega * t)
        c = omega * math.cos(omega * t)
        pos = [center[j] + amp[j] * env * s for j in range(TOTAL_JOINTS)]
        vel = [amp[j] * env * c for j in range(TOTAL_JOINTS)]
        return pos, vel

    return Profile(
        name="带载老化 (圆柱抓握 %.2fHz)" % FREQ_HZ,
        kp=KP, kd=KD, iq=IQ,
        center=center,
        motion=motion,
        enabled=enabled,
        ramp_s=(1.0 / FREQ_HZ) * RAMP_FRACTION,
        stop_style="sine",
        feedforward=True,
        period_s=1.0 / FREQ_HZ,
        notes=notes,
    )


def main(argv=None) -> int:
    ap = build_argparser(__doc__.strip().splitlines()[0], examples="""用法:
  python %(prog)s --dry-run                    先看抓握/张开两端的角度
  python %(prog)s --depth 30 --amp-scale 0.4   浅握小幅试跑
  python %(prog)s --duration 2h                正式跑(默认 深度60%% 张开30%%)

握不拢 / 握太紧就调 --depth (粗圆柱→小, 细圆柱→大);
张不够开就调小 --release (0=完全伸直, 100=一直握着不松)。

四指外展摆动默认开(±20°, 绕手动撑开位)。--abd-swing-deg 0 = 二轴跟随抓握开合,
即上位机出厂默认。--abd-mode fan = 中心改成按设备限位张最大扇形。
""")
    ap.add_argument("--depth", type=int, default=60,
                    help="抓握深度%% (0=不抓 100=满深蜷曲), 默认 60")
    ap.add_argument("--release", type=int, default=30,
                    help="张开程度%% (0=完全伸直 100=一直握着), 默认 30")
    ap.add_argument("--abd-mode", choices=("manual", "fan"), default="manual",
                    help="四指外展**中心**在哪 (默认 manual=锁定当前手动撑开位)")
    ap.add_argument("--abd-swing-deg", type=float, default=20.0,
                    help="四指外展**摆幅**(度, 单侧, 逐指分级), 默认 20 = 二轴默认参与老化; "
                         "0 = 跟随抓握开合(上位机出厂默认)")
    args = ap.parse_args(argv)
    if not (0 <= args.depth <= 100) or not (0 <= args.release <= 100):
        ap.error("--depth / --release 必须在 0..100 之间")
    return run_multi(build_profile, args,
                     banner="===== WH120 带载老化 demo (圆柱抓握 %.2fHz) =====" % FREQ_HZ)


if __name__ == "__main__":
    sys.exit(main())
