#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单手指负载老化 —— 对齐上位机 `SingleFingerLoadAgingController`。

**场景**: 治具盒上只挂一根手指(挂负重块), 只让这根手指的 4 个轴逐个走满行程。
本 demo **默认跑所有在线手指** —— 治具盒上通常就只有一根在线, 正好是想要的;
整只手都在线时会 5 根一起跑(见下方警告)。`--finger` 可以再筛。
没被选中的关节不使能, 命令帧里给它们填保位值。

**动作**: 一圈 17 段的梯形序列, 段间用 smoothstep (3u²-2u³, 两端速度为 0) 插值,
跑完一圈接着从头再来, 无限循环:

    J1 ±45°  →  J3 ±45°  →  J4 ±45°  →  (J1 屈到 +90° 保持) J2 外展 ±35°

J2(外展轴) 段要先把 J1 屈到 +90° 停住再摆 —— 这两个角度都是治具盒单指实测可达值
(治具上只挂一根指, 不存在相邻指剐蹭); ±35° 也落在四指 mcp_abd 的设计行程 ±40° 内。

**目标角会被钳进该关节自己的限位, 且不留内缩余量** —— +90° 正好是四指 mcp_flex
的设计上限, 内缩 5° 会把实测可达行程削掉。钳位主要是防拇指: cmc_flex 上限只有
+74°, 硬发 +90° 会每圈堵转 3 秒顶死机械限位。

**停机特别处理 —— 扭矩上限缓释放**:
disable 是二值的(直接断 PWM), 挂着负重块的手指会自由落体砸下去。所以回零到位后
先把 effort_limit 分 10 档、2 秒内缓降到 0A, 让重力在一个衰减的扭矩天花板下把手指
慢慢带下去, 再断使能。effort_limit 的 SET 只写固件 RAM 镜像 + 热同步电流环,
不写 flash, 所以反复刷不磨 flash 寿命。
"""

from __future__ import annotations

import sys
from typing import List, Tuple

from aging_common import (
    D2R, R2D, TOTAL_JOINTS,
    FINGER_NAMES, HandRunner, JointLimits, Profile,
    build_argparser, finger_index, finger_of, joint_label, run_multi, smoothstep,
)

# ── 运动参数 (对应上位机「运动参数中心 → 单指负载老化」一行) ─────────────
KP, KD, IQ = 3.0, 0.1, 1.5      # DEFS[SingleFingerLoadAging] = {3.0, 0.10, 1.5}

ANGLE_RAD = 45.0 * D2R          # J1/J3/J4 的对称摆幅
J1_FLEX_RAD = 90.0 * D2R        # J2 段里 J1 先屈到这个角度停住
ABD_RAD = 35.0 * D2R            # J2 外展摆幅
SEGMENT_S = 1.6                 # 单程 45° 的时长
SWING_S = SEGMENT_S * 2.0       # 双程 (+45 → -45) 的时长, 保持角速度一致
HOLD_S = 0.2                    # 保持段
INIT_ZERO_S = 2.5               # 每圈开头的「回零并稳定」段

RELEASE_S = 2.0                 # 缓释放总时长
RELEASE_STEPS = 10
RELEASE_FLOOR_A = 0.0           # 缓释放地板值; 治具几何上会磕到东西的话可提前收手

# (J1, J2, J3, J4 目标角 rad, 段时长 s, 说明) —— 逐条对齐 startSequence()
STEPS: Tuple[Tuple[Tuple[float, float, float, float], float, str], ...] = (
    ((0.0, 0.0, 0.0, 0.0),                    INIT_ZERO_S, "回零并稳定"),
    ((0.0, 0.0, 0.0, 0.0),                    HOLD_S,      "零位稳定保持"),
    (( ANGLE_RAD, 0.0, 0.0, 0.0),             SEGMENT_S,   "J1 -> +45°"),
    ((-ANGLE_RAD, 0.0, 0.0, 0.0),             SWING_S,     "J1 +45° -> -45°"),
    ((0.0, 0.0, 0.0, 0.0),                    SEGMENT_S,   "J1 回零"),
    ((0.0, 0.0,  ANGLE_RAD, 0.0),             SEGMENT_S,   "J3 -> +45°"),
    ((0.0, 0.0, -ANGLE_RAD, 0.0),             SWING_S,     "J3 +45° -> -45°"),
    ((0.0, 0.0, 0.0, 0.0),                    SEGMENT_S,   "J3 回零"),
    ((0.0, 0.0, 0.0,  ANGLE_RAD),             SEGMENT_S,   "J4 -> +45°"),
    ((0.0, 0.0, 0.0, -ANGLE_RAD),             SWING_S,     "J4 +45° -> -45°"),
    ((0.0, 0.0, 0.0, 0.0),                    SEGMENT_S,   "J4 回零"),
    ((J1_FLEX_RAD, 0.0, 0.0, 0.0),            SWING_S,     "J1 -> +90°"),
    ((J1_FLEX_RAD, 0.0, 0.0, 0.0),            HOLD_S,      "J1 +90° 稳定保持"),
    ((J1_FLEX_RAD,  ABD_RAD, 0.0, 0.0),       SEGMENT_S,   "J1 保持 +90°, J2 -> +35°"),
    ((J1_FLEX_RAD, -ABD_RAD, 0.0, 0.0),       SWING_S,     "J1 保持 +90°, J2 +35° -> -35°"),
    ((J1_FLEX_RAD, 0.0, 0.0, 0.0),            SEGMENT_S,   "J1 保持 +90°, J2 回零"),
    ((0.0, 0.0, 0.0, 0.0),                    SWING_S,     "J1 从 +90° 回零"),
)

CYCLE_S = sum(d for _, d, _ in STEPS)


def build_profile(ctx: HandRunner) -> Profile:
    limits: JointLimits = ctx.limits
    args = ctx.args

    # 默认跑**所有在线手指**(治具盒上通常只挂一根, 那就只有它在线); --finger 再筛。
    online = ctx.online_idxs()
    fingers = sorted({finger_of(k) for k in online})
    if args.finger:
        picked = {finger_index(n) for n in args.finger.split(",") if n.strip()}
        fingers = [f for f in fingers if f in picked]
        if not fingers:
            raise ValueError("--finger 选的手指一个都不在线")
    if not fingers:
        raise RuntimeError("没有在线的手指")

    # 只使能这些手指里**真正在线**的轴 —— 治具上可能一根手指也不是四轴全在。
    enabled = [False] * TOTAL_JOINTS
    for k in online:
        if finger_of(k) in fingers:
            enabled[k] = True

    # 预先算好每段的 (起点, 终点, 起始时刻) —— 起点 = 上一段的终点; 第 0 段的
    # 上一段是本圈最后一段, 其终点是全零, 与回零斜坡末尾连续。
    bounds: List[Tuple[Tuple[float, ...], Tuple[float, ...], float, float]] = []
    t_acc = 0.0
    prev = STEPS[-1][0]
    for target, dur, _label in STEPS:
        bounds.append((prev, target, t_acc, dur))
        t_acc += dur
        prev = target

    # 名义角 → 各手指各轴的钳位后目标。钳位在下发这一步做, 插值仍在名义空间算,
    # 否则多手指组时同一份名义目标要同时表示被削掉的不同值, ramp 起点就错了。
    def motion(t: float, env: float):
        u = t % CYCLE_S
        src, dst, t0, dur = bounds[0]
        for src_i, dst_i, t0_i, dur_i in bounds:
            if t0_i <= u < t0_i + dur_i:
                src, dst, t0, dur = src_i, dst_i, t0_i, dur_i
                break
        s = smoothstep((u - t0) / dur)
        nominal = [src[j] + (dst[j] - src[j]) * s for j in range(4)]

        pos = [0.0] * TOTAL_JOINTS
        for f in fingers:
            for j in range(4):
                k = f * 4 + j
                pos[k] = limits.clamp(k, nominal[j])
        return pos, [0.0] * TOTAL_JOINTS

    notes = [
        "手指组: %s (在线 %d 轴), 一圈 %.1fs"
        % (", ".join(FINGER_NAMES[f] for f in fingers),
           sum(1 for e in enabled if e), CYCLE_S),
        "停机会先回零, 再把扭矩上限 %.2fA → %.2fA 缓降 %.1fs, 最后断使能 —— "
        "挂负重块时别强杀进程, 否则手指自由落体。" % (IQ, RELEASE_FLOOR_A, RELEASE_S),
    ]
    if len(fingers) > 1:
        # 这套序列的角度是「治具上只挂一根指」实测出来的, 前提是没有相邻指。
        notes.append("⚠ 同时跑 %d 根手指: 本序列的 +90°/±35° 是**单指治具**上标定的, "
                     "整手一起跑时拇指与食指可能剐蹭 —— 先 --amp-scale 0.3 确认。"
                     % len(fingers))
    # 提前把「哪些轴的名义行程会被限位削掉」算出来告诉操作员 —— 主要是拇指:
    # cmc_flex 上限只有 +74°, 序列里的 +90° 会被削, 不提示的话看着像动作没做全。
    for f in fingers:
        clipped = []
        for j in range(4):
            k = f * 4 + j
            want_lo = min(step[0][j] for step in STEPS)
            want_hi = max(step[0][j] for step in STEPS)
            got_lo, got_hi = limits.clamp(k, want_lo), limits.clamp(k, want_hi)
            if abs(got_lo - want_lo) > 1e-6 or abs(got_hi - want_hi) > 1e-6:
                clipped.append("%s %.0f~%.0f° → %.0f~%.0f°"
                               % (joint_label(k), want_lo * R2D, want_hi * R2D,
                                  got_lo * R2D, got_hi * R2D))
        if clipped:
            notes.append("限位削幅(%s): %s" % (FINGER_NAMES[f], "; ".join(clipped)))

    return Profile(
        name="单手指负载老化 (%s)" % ",".join(FINGER_NAMES[f] for f in fingers),
        kp=KP, kd=KD, iq=IQ,
        center=[0.0] * TOTAL_JOINTS,
        motion=motion,
        enabled=enabled,
        ramp_s=0.0,                 # 梯形序列自带 2.5s 回零起步, 不需要振幅渐入
        stop_style="freeze",
        feedforward=False,          # 与上位机滑条通道一致: 只发位置, 不发速度/前馈
        freeze_ramp_s=SEGMENT_S,
        release_s=RELEASE_S,
        release_steps=RELEASE_STEPS,
        release_floor_a=RELEASE_FLOOR_A,
        period_s=CYCLE_S,
        notes=notes,
    )


def main(argv=None) -> int:
    ap = build_argparser(__doc__.strip().splitlines()[0], examples="""用法:
  python %(prog)s --dry-run                    先看每根手指的目标角和限位削幅
  python %(prog)s                              跑全部在线手指(治具盒上通常就一根)
  python %(prog)s --finger index               只跑食指
  python %(prog)s --finger index,middle        跑食指 + 中指

挂负重块时: 停机会先回零, 再把扭矩上限 2 秒内缓降到 0A 才断使能。
**别强杀进程**, 否则手指自由落体砸下去。
""")
    ap.add_argument("--finger", metavar="LIST",
                    help="只跑指定手指 (默认跑全部在线手指)。逗号分隔, "
                         "thumb/index/middle/ring/pinky")
    args = ap.parse_args(argv)
    return run_multi(build_profile, args,
                     banner="===== WH120 单手指负载老化 demo (一圈 %.1fs) =====" % CYCLE_S)


if __name__ == "__main__":
    sys.exit(main())
