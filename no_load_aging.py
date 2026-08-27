#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""空载老化 (整手扫频老化) —— 对齐上位机 `SweepWorker::doStartAging`。

**动作**: 整手 20 个关节各自在自己的机械行程内做全幅正弦往复, 0.3 Hz, 无限循环。

    pos = center + amp · env · sin(ωt + φ)
    vel = amp · env · ω · cos(ωt + φ)          (前馈 effort = 0.5 · vel)

**防撞** 全靠相位错开 + 逐轴限幅, 逐条对齐上位机(每条都是真机撞出来的经验):
  - 拇指 J1/J2 (CMC 屈曲 + 外展) 相位 π —— 四指往掌心蜷时拇指外展躲开;
  - 四指 J3/J4 (pip/dip) 相位 π, 与 J1 (mcp_flex) 反相 —— 形成「勾」形而不是
    「握拳」, 指尖被推到远端不戳掌心, 相邻指尖错位不剐蹭;
  - 拇指 J2 两端非对称内缩 (外展端 25°, 对掌端 30°) —— 正弦峰值会过冲 ~15°;
  - 拇指 J3 / 四指 pip 的 max 端各内缩 10° —— 防极限蜷曲贴掌心;
  - 四指 J2 (mcp_abd, 外展) 的**中心**锁一个撑开的扇形姿态把四指分开。

**四指外展 = 中心 + 摆幅两件独立的事**, 与上位机的两个开关一一对应:
  --abd-mode  决定**中心**在哪 (对应 fourFingerAbdManual):
      manual (默认, = 上位机出厂默认)
          锁定启动瞬间的实际姿态。**操作员必须先手动把四指撑成扇形再启动**,
          否则四指并拢着蜷曲会互相剐蹭。上位机默认就是这个模式(绕开外展方向
          跨固件版本不一致的问题, 工人摆对就行)。
      preset  摆到一组实测采集的扇形姿态(左右手各一套), 不依赖操作员。
      fan     按本手限位自动张到最大扇形 (旧名 swing, 仍可用)。
  --abd-swing-deg 决定**摆幅** (对应 fourFingerAbdSwingEnabled + Deg):
      绕上述中心做正弦, 逐指分级(食/中满摆, 无名 ×0.6, 小指 ×0.5)。
      **本工程默认 20°**(上位机出厂默认是关的) —— 让二轴也吃到老化行程。
      给 0 = 二轴只保位不动, 即复刻上位机出厂行为。
"""

from __future__ import annotations

import math
import sys
from typing import List

from aging_common import (
    AGING_MARGIN_RAD, D2R, PI, R2D, TOTAL_JOINTS,
    HandRunner, JointLimits, Profile,
    build_argparser, flat_to_nid, joint_label, run_multi, slot_of,
)

# ── 运动参数 (对应上位机「运动参数中心 → 扫频验证/空载老化」一行) ──────────
FREQ_HZ = 0.3          # MotionParamStore::sweepAgingFreqHz 默认值
CYCLES = 1             # SWEEP_TABLE[1].cycles —— 单段循环, 只用来定渐入时长
KP, KD, IQ = 5.0, 0.1, 1.0     # DEFS[SweepAging] = {5.0, 0.10, 1.0}
RAMP_FRACTION = 0.25   # 首段振幅渐入占段长的比例

# ── 逐轴限幅 (SweepWorker::doStartAging 里的同名常量) ────────────────────
THUMB_J2_OUTER_TRIM_RAD = 25.0 * D2R   # 拇指 CMC 外展端内缩
THUMB_J2_INNER_TRIM_RAD = 30.0 * D2R   # 拇指 CMC 对掌端内缩
THUMB_J3_MAX_TRIM_RAD = 10.0 * D2R     # 拇指 MCP 蜷曲端内缩
FOUR_FINGER_PIP_MAX_TRIM_RAD = 10.0 * D2R

# ── 四指外展 (mcp_abd) 相关 ──────────────────────────────────────────────
ABD_FLAT_IDX = (5, 9, 13, 17)          # NID 7 / 12 / 17 / 22, 顺序 = 食/中/无名/小
# 右手扇形预设 (2026-05-22 用户手摆好后 199 帧广播平均, std=0)
ABD_PRESET_RIGHT = (-0.47438, -0.12469, +0.22880, +0.62601)
# 左手扇形预设 (2026-06-08 从相序正确的左手实测采集; **不是**右手取反)
ABD_PRESET_LEFT = (-0.49218, -0.07679, +0.25307, +0.53756)
ABD_SWING_SCALE = (1.0, 1.0, 0.6, 0.5)     # 逐指分级: 外侧减幅防剐蹭
FAN_SPREAD_SIGN = (-1.0, -1.0, +1.0, +1.0)  # 大扇形方向
FAN_SPREAD_FRAC = (1.0, 0.333, 0.333, 1.0)  # 食/小最外, 中/无名居中
FAN_MARGIN_RAD = 5.0 * D2R


def build_profile(ctx: HandRunner) -> Profile:
    limits: JointLimits = ctx.limits
    args = ctx.args
    is_left = str(ctx.handedness).lower().startswith("l")

    center = [0.0] * TOTAL_JOINTS
    amp = [0.0] * TOTAL_JOINTS
    phase = [0.0] * TOTAL_JOINTS
    enabled = [False] * TOTAL_JOINTS

    # ── 1. 基线: 每个关节在自己的限位内全幅摆, 两端各留 5° 余量 ──────────
    for k in range(TOTAL_JOINTS):
        ca = limits.center_amp(k, AGING_MARGIN_RAD)
        if ca is None:
            continue
        center[k], amp[k] = ca
        enabled[k] = True
        nid = flat_to_nid(k)
        # 相位防撞。拇指 J1/J2 给 π (左右手都给 —— 2026-06-08 在相序正确的左手上
        # 实测: 给 0 会让拇指与四指同相一起往掌心 → 撞)。
        if nid in (1, 2):
            phase[k] = PI
        elif nid >= 5 and slot_of(k) in (2, 3):
            phase[k] = PI

    # ── 2. 拇指 J2: 两端不对称内缩 → center/amp 按生效边界重算 ───────────
    _retrim(limits, center, amp, k=1,
            lo_trim=THUMB_J2_OUTER_TRIM_RAD, hi_trim=THUMB_J2_INNER_TRIM_RAD)
    # ── 3. 拇指 J3: max 端多缩 10° ───────────────────────────────────────
    _retrim(limits, center, amp, k=2,
            lo_trim=AGING_MARGIN_RAD, hi_trim=THUMB_J3_MAX_TRIM_RAD)
    # ── 4. 四指 pip (J3): max 端多缩 10° ─────────────────────────────────
    for k in (6, 10, 14, 18):
        _retrim(limits, center, amp, k=k,
                lo_trim=AGING_MARGIN_RAD, hi_trim=FOUR_FINGER_PIP_MAX_TRIM_RAD)

    # ── 5. 四指外展 (J2): 三选一 ─────────────────────────────────────────
    notes: List[str] = []
    mode = args.abd_mode
    swing_rad = args.abd_swing_deg * D2R
    preset = ABD_PRESET_LEFT if is_left else ABD_PRESET_RIGHT
    # **中心和摆幅是两件独立的事** —— 对齐上位机: fourFingerAbdManual 只决定中心
    # (手动位 / 扇形预设 / 设备限位大扇形), fourFingerAbdSwingEnabled+Deg 只决定摆幅,
    # 两者任意组合 (SweepWorker.cpp doStartAging 里 ampK 与 manualAbd 分开算)。
    # 早先把 manual/preset 硬编成 amp=0, 表达不出"锁定手动位 + 在该位置上摆"这一档,
    # 而那恰恰是上位机默认状态下打开摆动开关的那条路。
    for i, k in enumerate(ABD_FLAT_IDX):
        phase[k] = 0.0
        amp[k] = swing_rad * ABD_SWING_SCALE[i]
        if mode == "manual":
            center[k] = ctx.actual[k]          # 锁定操作员摆好的当前位置
        elif mode == "preset":
            center[k] = preset[i]
        else:                                   # fan (旧名 swing): 按设备限位张最大
            rng = limits.get(k)
            if rng is None:
                amp[k] = 0.0
                continue
            mid = (rng[0] + rng[1]) * 0.5
            reach = max(0.0, (rng[1] - rng[0]) * 0.5 - FAN_MARGIN_RAD)
            center[k] = mid + FAN_SPREAD_SIGN[i] * FAN_SPREAD_FRAC[i] * reach

    # 摆幅按各指**可用余量**自动夹住: 中心离限位越近, 能摆的就越小。
    # 不夹的话, preset/fan 这种把中心推到扇形外侧的模式配上大摆幅, 每周期都有一段
    # 被固件夹在限位上(跟踪率掉到 70~80%, 状态行报限位), 等于白摆还发热。
    clamped = []
    for i, k in enumerate(ABD_FLAT_IDX):
        rng = limits.get(k)
        if rng is None or amp[k] <= 0.0:
            continue
        room = max(0.0, min(center[k] - rng[0], rng[1] - center[k]) - FAN_MARGIN_RAD)
        if amp[k] > room:
            clamped.append("%s %.0f→%.0f°" % (joint_label(k), amp[k] * R2D, room * R2D))
            amp[k] = room

    pose = ", ".join("%.0f" % (center[k] * R2D) for k in ABD_FLAT_IDX)
    if mode == "manual":
        notes.append("四指外展中心 = 手动撑开模式: 已锁定当前姿态 [%s]°, "
                     "**启动前请先把四指手动撑成扇形**, 否则蜷曲时会互相剐蹭。" % pose)
    elif mode == "preset":
        notes.append("四指外展中心 = %s扇形预设 [%s]°"
                     % ("左手" if is_left else "右手", pose))
    else:
        notes.append("四指外展中心 = 按设备限位自动张最大 [%s]°" % pose)
    if swing_rad > 0.0:
        notes.append("四指外展摆动 = 绕上述中心 ±%.1f° 逐指分级 (食/中满摆·无名 %.1f·小指 %.1f) "
                     "—— 摆幅须在台架上从小往大标定, 别直接上大值。"
                     % (args.abd_swing_deg, ABD_SWING_SCALE[2], ABD_SWING_SCALE[3]))
    else:
        notes.append("四指外展摆动 = 关 (--abd-swing-deg 0), 二轴只保位不动")
    if clamped:
        notes.append("四指外展摆幅被限位余量夹小: %s (中心离限位太近, 摆不开)"
                     % ", ".join(clamped))
    notes.append("手型 %s, 拇指 J1/J2 与四指反相 π 防撞" % ctx.handedness)

    omega = 2.0 * PI * FREQ_HZ

    def motion(t: float, env: float):
        pos = [0.0] * TOTAL_JOINTS
        vel = [0.0] * TOTAL_JOINTS
        for j in range(TOTAL_JOINTS):
            a = amp[j]
            if a == 0.0:
                pos[j] = center[j]
                continue
            th = omega * t + phase[j]
            pos[j] = center[j] + a * env * math.sin(th)
            vel[j] = a * env * omega * math.cos(th)
        return pos, vel

    segment_s = CYCLES / FREQ_HZ
    return Profile(
        name="空载老化 (整手扫频 %.2fHz)" % FREQ_HZ,
        kp=KP, kd=KD, iq=IQ,
        center=center,
        motion=motion,
        enabled=enabled,
        ramp_s=segment_s * RAMP_FRACTION,
        stop_style="sine",
        feedforward=True,
        period_s=1.0 / FREQ_HZ,
        notes=notes,
    )


def _retrim(limits: JointLimits, center: List[float], amp: List[float],
            k: int, lo_trim: float, hi_trim: float) -> None:
    """按不对称内缩重算某个关节的 (center, amp)。

    两端内缩不一样时 center 就不再是 (min+max)/2 —— 不重算的话正弦仍绕旧中心摆,
    会直接突破新边界。上位机在拇指 J2/J3 与四指 pip 上都做了这一步。
    """
    rng = limits.get(k)
    if rng is None:
        return
    lo = rng[0] + lo_trim
    hi = rng[1] - hi_trim
    if hi > lo:
        center[k] = (lo + hi) * 0.5
        amp[k] = (hi - lo) * 0.5


def main(argv=None) -> int:
    ap = build_argparser(__doc__.strip().splitlines()[0], examples="""用法:
  python %(prog)s --dry-run              先看行程和限位余量, 不使能不运动
  python %(prog)s --amp-scale 0.2        小幅试跑, 确认四指不剐蹭
  python %(prog)s --duration 2h          正式跑 2 小时后自动停机

四指外展 = **中心** + **摆幅** 两件独立的事 (与上位机的两个开关一一对应):
  --abd-mode       中心在哪 (默认 manual)
      manual   锁定启动瞬间的姿态 —— **启动前请先手动把四指撑成扇形**
      preset   摆到实测采集的扇形预设(左右手各一套), 不依赖操作员
      fan      按本手限位张到最大扇形 (旧名 swing, 仍可用)
  --abd-swing-deg  绕上述中心摆多大 (默认 20°, 逐指分级; 给 0 = 二轴不动)
""")
    ap.add_argument("--abd-mode", choices=("manual", "preset", "fan", "swing"),
                    default="manual",
                    help="四指外展**中心**在哪 (默认 manual, 与上位机出厂默认一致)")
    ap.add_argument("--abd-swing-deg", type=float, default=20.0,
                    help="四指外展**摆幅**(度, 单侧, 逐指分级), 默认 20 = 二轴默认参与老化; "
                         "0 = 二轴只保位不动(上位机出厂默认)")
    args = ap.parse_args(argv)
    return run_multi(build_profile, args,
                     banner="===== WH120 空载老化 demo (整手扫频 %.2fHz) =====" % FREQ_HZ)


if __name__ == "__main__":
    sys.exit(main())
