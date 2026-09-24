#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""位控性能QC —— 对齐上位机 `PosPerfQcWorker`。

空载老化前的一道**判合格**工序(产线顺序: 相序检查 → 位控性能QC → 空载老化)。
不是老化, 一只手跑完约 3~5 分钟, 产出每指 PASS/FAIL。

**逐指测, 每指两趟**:

  第一趟 跟踪延迟 —— 该指各轴同时走 1 Hz 正弦(相位错开 90° 防同时蜷进掌心),
      用**锁相**(lock-in)从反馈里解出位置滞后的相位, 折成毫秒。
      门限: 每轴 < 45 ms。
  第二趟 静态静差 —— 逐轴单独推到行程两端保持, 其余轴守在中心;
      固件**没有积分项**, 所以上位机侧跑一个软件积分(ki=5, 力矩钳 0.3A)把静差压掉,
      压不掉的残差就是这一轴的静差。四轴静差按力臂折算成**指尖偏差**。
      门限: 指尖上/下端都 < 2 mm。

  整指合格 ⟺ 各轴延迟都过 **且** 指尖两端都过。整手 PASS ⟺ 所测手指全合格。

**为什么逐指而不是整手一起**: 20 轴齐动的母线电流会把扭矩顶到限流, 限流下量到的
延迟和静差全不可信; 一次 enable 20 个关节还偶发会话卡死。上位机同样逐指跑。

**四指二轴(mcp_abd)不测** —— 锁在中心不动, 跟上位机一致(`isSwept()` 里
`!(finger>=1 && j==1)`)。只有拇指测全 4 轴。指尖折算里它按外展分量单独算。

================  与上位机的两处差异, 上产线前必须先对一遍  ================

1. **限位读不到真值。** 上位机逐轴读固件的 `od/ps_final_pos_{down,up}_rad` 拿这只手
   的真限位, 公开 SDK 没有原始对象字典读接口(`WujiHand2` 只有
   effort_limit/error_code/status_word), 所以本脚本用**设计限位**
   (`joint_limits.json`)或 `limits_<SN>.json`。

   后果比老化严重得多: 设计值比真限位宽时命令角会过界被固件夹住, 那一段量到的
   「静差」是「命令角 − 限位角」而不是伺服误差, 拿去判就是**误判不合格**。
   所以本脚本在每段测量里盯 `position_limit_active`, 一旦夹住就把该指判成
   **判不了(INVALID)**, 而不是不合格 —— 宁可要人来看一眼, 不能误杀好手。

   先跑一轮 `python single_joint_aging.py --amp-scale 1.0 --duration 75s
   --export-limits` 标定出 `limits_<SN>.json`, 之后本脚本按 SN 自动认。

2. **45 ms 这个门限里含约 8 ms 的 Qt bridge 测量路径偏移。** 上位机源码原话:
   `45 = real PD lag ~31ms + ~8ms Qt-bridge measurement-path offset (vs SDK)`——
   括号里的 `vs SDK` 就是说 SDK 这条路径**没有**那 8 ms。所以照搬 45 会比上位机
   **松约 8 ms**; 要一样严就 `--lag-ms-max 37`。

   默认仍取 45(产线规格上写的那个数)。**这个数必须在同一只手上与上位机对一遍**
   再拿去当判据 —— 跟 README 里四指外展摆幅一样, 是要现场标的量, 不是照抄的常数。
"""

from __future__ import annotations

import json
import math
import sys
import threading
from typing import Dict, List, Optional, Sequence, Tuple

from aging_common import (
    LIM_CUR, LIM_POS, PI, R2D, TOTAL_JOINTS,
    FINGER_NAMES, FINGER_NAMES_CN, HandRunner, JointLimits, Profile,
    build_argparser, finger_index, flat_to_nid, joint_label, run_multi,
)

# ── 增益 / 判据 (逐条对应 PosQcParams 的同名字段) ─────────────────────────
KP, KD, IQ = 8.0, 0.2, 1.5     # 比老化硬得多 —— 要量的是伺服能力, 不是耐久
KI, ICLAMP = 5.0, 0.3          # 软件积分: 固件 MIT 没有 ki, 静差只能在上位机侧补
MARGIN = 0.9                   # 行程取半程的这个比例 (不是老化那样按角度内缩)
FREQ_HZ = 1.0                  # 锁相正弦频率
S2_SHRINK = 0.05               # 拇指 J2 负侧内缩比例 (躲掌心)
LAG_MS_MAX = 45.0              # 判据①: 跟踪延迟上限, 见模块顶部差异 2
TIP_MM_MAX = 2.0              # 判据②: 指尖静差上限 (整手指尖 ±2mm 精度指标)

# ── 时序 (PosPerfQcWorker.cpp 匿名 namespace 里的同名常量) ────────────────
RAMP_S = 1.5                   # 各段之间的就位斜坡
LAG_SETTLE_S = 2.0             # 渐入完再稳这么久才开始积分
LAG_CYCLES = 4                 # 锁相积分的周期数 (整周期, 见 _lag_sweep)
STAT_SET_S = 3.0               # 静态段: 稳定 + 让积分吃掉静差
STAT_MEAS_S = 0.6              # 静态段: 取均值的窗口
PHASE_STEP = PI / 2.0          # 同指各轴的相位错开量 (防同时蜷进掌心)

#: 反馈幅度低于命令幅度这个比例 → 锁相解出来的相位是噪声, 不能当延迟用。
#: 不设这道闸的话, 一个完全不动的轴会解出一个随机相位, 有一半概率 < 45ms 而**假合格**。
LAG_TRACK_MIN = 0.2

#: 各轴到指尖的**力臂**(mm), 按 flat 下标。来自上位机 `L_ARM[]`(hand2_beta URDF)。
#: ⚠ 和 README §10.1 老化判据里的「轴间距 + 累加链」是**两套不同的几何**:
#: 那边算的是「根部误差带着远端一起转」的累加偏差, 这里算的是每轴静差各自对指尖的
#: 力臂投影。两者不能互相顶替, 也别混用数值。
L_ARM = (
    108.1, 97.2, 64.8, 29.8,   # thumb  J1..J4
    106.4, 91.9, 54.3, 24.8,   # index
    106.4, 91.9, 54.3, 24.8,   # middle
    106.4, 91.9, 54.3, 24.8,   # ring
    100.4, 85.9, 52.3, 24.8,   # pinky
)

PASS, FAIL, INVALID = "PASS", "FAIL", "INVALID"

#: SN → 本手结果。一拖多时各线程往里写, `main()` 跑完统一汇总 + 定退出码。
_RESULTS: Dict[str, dict] = {}
_RESULTS_LOCK = threading.Lock()


# ──────────────────────────────────────────────────────────────────────────
# 行程
# ──────────────────────────────────────────────────────────────────────────

def is_swept(finger: int, slot: int) -> bool:
    """这个轴要不要测。四指 J2(mcp_abd) 锁死不测, 拇指测全 4 轴 —— 对齐上位机。"""
    return not (finger >= 1 and slot == 1)


def finger_span(limits: JointLimits, finger: int, online: Sequence[bool],
                margin: float) -> Tuple[List[bool], List[float], List[float]]:
    """该指四个轴的 (要不要测, 中心, 振幅)。

    `center = (下限 + 上限)/2`, `amp = 半程 × margin` —— 注意**不是**老化那套
    「两端各内缩 5°」: 按比例缩在短行程轴上留的余量更小, 这是上位机的选择,
    照搬(它配的是真限位; 我们配设计限位时靠限位夹住检测兜底)。
    """
    swept = [False] * 4
    center = [0.0] * 4
    amp = [0.0] * 4
    for j in range(4):
        k = finger * 4 + j
        if not is_swept(finger, j) or not online[k]:
            continue
        rng = limits.get(k)
        if rng is None:
            continue
        lo, hi = rng
        c = (lo + hi) * 0.5
        a = (hi - lo) * 0.5 * margin
        if finger == 0 and j == 1:
            # 拇指 J2(CMC 外展)负侧内缩, 躲掌心。上位机是对**端点值**乘 (1-shrink):
            # 端点为负时等于往里收, 正好是想要的; 逐字照搬, 别自己改成减一个角度。
            top = c + a
            bot = (c - a) * (1.0 - S2_SHRINK)
            c, a = (bot + top) * 0.5, (top - bot) * 0.5
        if a <= 0.0:
            continue
        swept[j], center[j], amp[j] = True, c, a
    return swept, center, amp


# ──────────────────────────────────────────────────────────────────────────
# 第一趟: 锁相测跟踪延迟
# ──────────────────────────────────────────────────────────────────────────

def lag_sweep(runner: HandRunner, prof: Profile, finger: int,
              swept: Sequence[bool], center: Sequence[float], amp: Sequence[float],
              freq: float, cycles: int) -> Tuple[bool, List[float], List[float],
                                                 List[int]]:
    """该指各轴同时走正弦, 锁相解出位置滞后。

    返回 (跑完没被打断, 逐轴延迟 ms, 逐轴跟踪率, 逐轴限幅掩码)。

    **锁相**: 命令 `c + A·sin(θ)`, 反馈约 `c + A'·sin(θ-φ)`。把反馈分别乘
    `sin(θ)` / `cos(θ)` 累加, 正交分量就出来了 —— `φ = atan2(-Σcos, Σsin)`,
    再按频率折成毫秒。比「找峰值时刻求差」稳得多: 它用上了整段每一个采样点,
    单点噪声和偶发掉帧都被平均掉。

    **相位错开 90°** 是防撞(四轴同时往掌心蜷会戳到掌心), 不影响测量: 每个轴的
    锁相用的是它自己的 θ。

    积分窗口取**整数个周期**, 这样 `Σsin`/`Σcos` 为 0, 中心位那一项自然消掉。
    但节拍有抖动, 窗口末尾总会多出小半拍 —— 所以这里再显式减掉直流分量
    (`Σ(q-q̄)·sin`), 不依赖窗口刚好整周期。上位机没做这一步, 属同向的精度改进,
    窗口准时时两者结果一致。
    """
    w = 2.0 * PI * freq
    # 拇指起步方向相反 —— 对齐上位机的 `sgn = (f == 0) ? -1.0 : 1.0`。
    sgn = -1.0 if finger == 0 else 1.0

    order: Dict[int, int] = {}
    for j in range(4):
        if swept[j]:
            order[j] = len(order)

    t_env = 1.0 / freq                       # 一个周期把振幅渐入
    meas_start = t_env + LAG_SETTLE_S
    meas_end = meas_start + cycles / freq

    # 锁相累加器: q = 反馈 × sgn
    s_q = [0.0] * 4
    s_qs = [0.0] * 4
    s_qc = [0.0] * 4
    s_s = [0.0] * 4
    s_c = [0.0] * 4
    n = [0] * 4
    lim = [0] * 4
    zeros = [0.0] * TOTAL_JOINTS

    def traj(t: float):
        env = min(t / t_env, 1.0) if t_env > 0.0 else 1.0
        pos = [0.0] * TOTAL_JOINTS
        for j in range(4):
            if not swept[j]:
                continue
            th = w * t - order[j] * PHASE_STEP
            pos[finger * 4 + j] = center[j] + sgn * amp[j] * env * math.sin(th)
        if t >= meas_start:
            for j in range(4):
                if not swept[j]:
                    continue
                k = finger * 4 + j
                th = w * t - order[j] * PHASE_STEP
                st, ct = math.sin(th), math.cos(th)
                q = runner.actual[k] * sgn
                s_q[j] += q
                s_qs[j] += q * st
                s_qc[j] += q * ct
                s_s[j] += st
                s_c[j] += ct
                n[j] += 1
                lim[j] |= runner.limit_flags(flat_to_nid(k))
        return pos, zeros                    # 纯位置通道: 带速度前馈会把延迟测小

    ok = runner.run_phase(prof, traj, meas_end,
                          "  [1/2] 跟踪延迟 %.2gHz × %d 周期" % (freq, cycles))

    lag_ms = [0.0] * 4
    track = [0.0] * 4
    for j in range(4):
        if not swept[j] or n[j] == 0:
            continue
        mean_q = s_q[j] / n[j]
        si = s_qs[j] - mean_q * s_s[j]       # Σ(q-q̄)·sin(θ)
        co = s_qc[j] - mean_q * s_c[j]       # Σ(q-q̄)·cos(θ)
        lag_ms[j] = math.atan2(-co, si) * R2D / (360.0 * freq) * 1000.0
        # 解出来的反馈幅度 (si ≈ n·A'·cosφ/2, co ≈ -n·A'·sinφ/2)
        a_meas = 2.0 * math.hypot(si, co) / n[j]
        track[j] = a_meas / amp[j] if amp[j] > 0.0 else 0.0
    return ok, lag_ms, track, lim


# ──────────────────────────────────────────────────────────────────────────
# 第二趟: 静态保持测静差
# ──────────────────────────────────────────────────────────────────────────

def measure_static(runner: HandRunner, prof: Profile, finger: int, j_meas: int,
                   swept: Sequence[bool], center: Sequence[float], target: float,
                   ki: float, iclamp: float) -> Optional[Tuple[float, int]]:
    """把 `j_meas` 单独推到 `target` 保持, 其余轴守中心, 量稳态静差(rad)。

    返回 (静差 = target − 实际均值, 该段限幅掩码); 被打断返回 None。

    固件 MIT 只有 P 和 D, 稳态一定留一截静差(重力/静摩擦顶着)。上位机的做法是在
    自己这一侧跑积分, 把积出来的力矩塞进命令帧的前馈通道 —— 所以量到的静差是
    「连积分都压不掉的那部分」, 而不是纯 P 控的静差。这里逐字照搬 ki / 钳位值,
    改动它们就不是同一个判据了。

    **逐轴单独测**: 一根手指四个轴串在一条链上, 同时推到端点的话每个轴的静差里
    都混着别人的负载。其余轴守在中心才隔离得开。
    """
    k_meas = finger * 4 + j_meas
    nid = flat_to_nid(k_meas)

    pose = [0.0] * TOTAL_JOINTS
    for j in range(4):
        if swept[j]:
            pose[finger * 4 + j] = center[j]
    pose[k_meas] = target

    # 斜坡过去: 顺手把上一轴从端点带回中心, 同时把本轴推到端点。
    if not runner.ramp_phase(prof, pose, RAMP_S):
        return None

    zeros = [0.0] * TOTAL_JOINTS
    st = {"integ": 0.0, "last": 0.0, "sum": 0.0, "n": 0, "lim": 0, "meas": False}

    def traj(t: float):
        dt = min(max(t - st["last"], 0.0), 0.05)   # 掉拍时别让积分猛跳
        st["last"] = t
        fb = runner.actual[k_meas]
        st["integ"] += (target - fb) * dt
        e = ki * st["integ"]
        # 钳位连**积分量本身**一起钳, 否则顶到上限后积分还在一路涨, 回程时要等很久
        # 才退得出饱和。(ki=0 是合法输入 —— 用来看纯 P 控的静差, 这时不会进这两支。)
        if e > iclamp:
            e, st["integ"] = iclamp, (iclamp / ki if ki else 0.0)
        elif e < -iclamp:
            e, st["integ"] = -iclamp, (-iclamp / ki if ki else 0.0)
        eff = [0.0] * TOTAL_JOINTS
        eff[k_meas] = e
        if st["meas"]:
            st["sum"] += fb
            st["n"] += 1
            st["lim"] |= runner.limit_flags(nid)
        return pose, zeros, eff

    if not runner.run_phase(prof, traj, STAT_SET_S):
        return None
    st["last"] = 0.0                                # t 每段从 0 起算, 但积分要接着走
    st["meas"] = True
    if not runner.run_phase(prof, traj, STAT_MEAS_S):
        return None

    mean = st["sum"] / st["n"] if st["n"] else runner.actual[k_meas]
    return target - mean, st["lim"]


def tip_mm(finger: int, swept: Sequence[bool], err: Sequence[float]) -> float:
    """四轴静差(rad) → 指尖偏差(mm)。屈伸分量与外展分量正交, 取模。"""
    flex = abd = 0.0
    for j in range(4):
        if not swept[j]:
            continue
        contrib = L_ARM[finger * 4 + j] * err[j]
        if j == 1:                 # J2 = 外展轴, 与屈伸不共面
            abd += contrib
        else:
            flex += contrib
    return math.hypot(flex, abd)


# ──────────────────────────────────────────────────────────────────────────
# 编排
# ──────────────────────────────────────────────────────────────────────────

def _lim_txt(mask: int) -> str:
    return " ".join(n for b, n in ((LIM_POS, "限位"), (LIM_CUR, "限流")) if mask & b)


def build_profile(ctx: HandRunner) -> Profile:
    args = ctx.args
    limits: JointLimits = ctx.limits
    margin = args.margin
    freq = args.freq if args.freq > 0.0 else FREQ_HZ

    online = list(ctx.online)
    fingers = sorted({k // 4 for k in ctx.online_idxs()})
    if args.finger:
        picked = {finger_index(s) for s in args.finger.split(",") if s.strip()}
        fingers = [f for f in fingers if f in picked]
        if not fingers:
            raise ValueError("--finger 选的手指一个都不在线")
    if not fingers:
        raise RuntimeError("没有在线的手指 —— 检查供电和网线")

    spans = {f: finger_span(limits, f, online, margin) for f in fingers}
    spans = {f: v for f, v in spans.items() if any(v[0])}
    if not spans:
        raise RuntimeError("这些手指一个可测轴都没有(限位表里查不到?)")
    fingers = sorted(spans)

    enabled = [False] * TOTAL_JOINTS
    for f in fingers:
        swept = spans[f][0]
        for j in range(4):
            if swept[j]:
                enabled[f * 4 + j] = True

    # ── 提示 ────────────────────────────────────────────────────────────
    n_axes = sum(1 for e in enabled if e)
    n_meas = sum(1 for k in range(TOTAL_JOINTS) if enabled[k] and limits.is_measured(k))
    notes = [
        "待测: %s, 共 %d 轴。四指 J2(外展)锁死不测, 只有拇指测全 4 轴 —— 同上位机。"
        % ("、".join(FINGER_NAMES_CN[f] for f in fingers), n_axes),
        "判据: 每轴跟踪延迟 < %.0f ms 且指尖静差上/下端都 < %.2f mm。"
        % (args.lag_ms_max, args.tip_mm_max),
        "逐指使能逐指测 —— 整手齐动会限流, 限流下量到的延迟和静差都不可信。",
    ]
    if n_meas < n_axes:
        # 注意: 下面这些 notes 会 print 到控制台, 而真机是 Windows(中文 code page
        # 常是 cp936/GBK) —— 只能用 GBK 编得出的字符, 别用 ⚠ / − / ⟺ 这类, 否则
        # 一行提示就能让整场 QC 崩在 UnicodeEncodeError 上。
        notes.append(
            "注意: %d/%d 个待测轴用的是**设计限位**, 不是这只手的实测限位(公开 SDK 读不到 "
            "`ps_final_pos`)。命令角可能过界被固件夹住, 那一段的静差是「命令角-限位角」"
            "不是伺服误差 —— 本脚本检出夹住会把该指判成「判不了」而不是不合格。"
            "先跑 `python single_joint_aging.py --amp-scale 1.0 --duration 75s "
            "--export-limits` 标定出 limits_<SN>.json。" % (n_axes - n_meas, n_axes))
    if abs(args.lag_ms_max - LAG_MS_MAX) < 1e-9:
        notes.append(
            "延迟门限 %.0f ms 是上位机那个数, 其中含约 8 ms 的 Qt bridge 测量路径偏移, "
            "而 SDK 这条路径没有 —— 所以实际比上位机**松约 8 ms**。要一样严用 "
            "`--lag-ms-max 37`。上产线前请在同一只手上与上位机对一遍。" % args.lag_ms_max)
    if args.amp_scale != 1.0:
        notes.append("注意: --amp-scale %.2f 行程不是满行程, **结果不作判据**(判定会"
                     "记成「判不了」)。只用来先确认方向和干涉。" % args.amp_scale)

    # ── --dry-run 用的名义波形: 第一趟的正弦包络 ─────────────────────────
    def motion(t: float, env: float):
        w = 2.0 * PI * freq
        pos = [0.0] * TOTAL_JOINTS
        vel = [0.0] * TOTAL_JOINTS
        for f in fingers:
            swept, center, amp = spans[f]
            sgn = -1.0 if f == 0 else 1.0
            for j in range(4):
                k = f * 4 + j
                if not swept[j]:
                    continue
                pos[k] = center[j] + sgn * amp[j] * env * math.sin(w * t)
        return pos, vel

    # ── 正式流程 ────────────────────────────────────────────────────────
    def sequence(runner: HandRunner, prof: Profile) -> None:
        run_qc(runner, prof, fingers, spans, freq)

    return Profile(
        name="位控性能QC (%d 指 %d 轴, %.2gHz)" % (len(fingers), n_axes, freq),
        kp=KP, kd=KD, iq=IQ,
        center=[0.0] * TOTAL_JOINTS,
        motion=motion,
        enabled=enabled,
        feedforward=False,          # 纯位置通道; 力矩由静态段自己给(见 _emit 的 eff)
        period_s=1.0 / freq,
        notes=notes,
        sequence=sequence,
        disable_all_on_exit=True,   # 逐指切使能 → 收尾时 mask 里只剩最后一指
    )


def run_qc(runner: HandRunner, prof: Profile, fingers: List[int],
           spans: Dict[int, Tuple[List[bool], List[float], List[float]]],
           freq: float) -> None:
    """`SequenceFn`: 逐指跑两趟, 出结论。"""
    args = runner.args
    scale = args.amp_scale
    zeros = [0.0] * TOTAL_JOINTS

    joint_rows: List[dict] = []
    finger_rows: List[dict] = []
    aborted = False

    runner.log("开始位控性能QC —— 逐指测, 每指约 %.0f 秒" % _finger_secs(freq, args))

    for f in fingers:
        if runner.stop_event.is_set():
            aborted = True
            break
        swept, center, amp_raw = spans[f]
        amp = [a * scale for a in amp_raw]
        runner.log("=== %s (%s) ===" % (FINGER_NAMES_CN[f], FINGER_NAMES[f]))

        enabled = [False] * TOTAL_JOINTS
        for j in range(4):
            if swept[j]:
                enabled[f * 4 + j] = True
        runner.scope(prof, enabled, label=" %s %d 轴" % (FINGER_NAMES_CN[f],
                                                         sum(1 for e in enabled if e)))

        center_pose = [0.0] * TOTAL_JOINTS
        for j in range(4):
            if swept[j]:
                center_pose[f * 4 + j] = center[j]
        if not runner.ramp_phase(prof, center_pose, RAMP_S, "  就位斜坡"):
            aborted = True
            runner.unscope(prof)
            break

        # ── 第一趟 ──────────────────────────────────────────────────────
        ok, lag_ms, track, lag_lim = lag_sweep(runner, prof, f, swept, center, amp,
                                               freq, args.cycles)
        if not ok:
            aborted = True
            runner.unscope(prof)
            break

        # ── 第二趟 ──────────────────────────────────────────────────────
        err_up = [0.0] * 4
        err_dn = [0.0] * 4
        stat_lim = [0] * 4
        for j in range(4):
            if not swept[j]:
                continue
            got = measure_static(runner, prof, f, j, swept, center,
                                 center[j] + amp[j], args.ki, args.iclamp)
            if got is None:
                aborted = True
                break
            err_up[j], m = got
            stat_lim[j] |= m
            got = measure_static(runner, prof, f, j, swept, center,
                                 center[j] - amp[j], args.ki, args.iclamp)
            if got is None:
                aborted = True
                break
            err_dn[j], m = got
            stat_lim[j] |= m
        if aborted:
            runner.unscope(prof)
            break

        # ── 判定 ────────────────────────────────────────────────────────
        up_mm = tip_mm(f, swept, err_up)
        dn_mm = tip_mm(f, swept, err_dn)
        max_lag = 0.0
        lag_bad: List[str] = []
        invalid: List[str] = []

        for j in range(4):
            k = f * 4 + j
            row = {
                "nid": flat_to_nid(k), "finger": f, "s": j + 1,
                "joint": joint_label(k), "swept": bool(swept[j]),
            }
            if swept[j]:
                mask = lag_lim[j] | stat_lim[j]
                lag_ok = lag_ms[j] < args.lag_ms_max
                tracked = track[j] >= LAG_TRACK_MIN
                max_lag = max(max_lag, lag_ms[j])
                if mask & LIM_POS:
                    invalid.append("%s 被限位夹住" % joint_label(k))
                if not tracked:
                    invalid.append("%s 只跟到命令的 %.0f%%, 解出的相位是噪声"
                                   % (joint_label(k), track[j] * 100.0))
                elif not lag_ok:
                    lag_bad.append("%s %.1fms" % (joint_label(k), lag_ms[j]))
                row.update({
                    "lagMs": round(lag_ms[j], 2),
                    "trackRatio": round(track[j], 3),
                    "steadyErrUpDeg": round(err_up[j] * R2D, 4),
                    "steadyErrDnDeg": round(err_dn[j] * R2D, 4),
                    "lagPass": bool(lag_ok and tracked),
                    "limitFlags": _lim_txt(mask),
                })
                if not tracked:
                    tag = "**跟踪不足, 延迟数无效**"
                elif not lag_ok:
                    tag = "**延迟超标**"
                else:
                    tag = "OK"
                runner.log("    %-14s 延迟 %6.1fms  跟踪 %3.0f%%  静差↑ %+6.2f° "
                           "↓ %+6.2f°  %s%s"
                           % (joint_label(k), lag_ms[j], track[j] * 100.0,
                              err_up[j] * R2D, err_dn[j] * R2D, tag,
                              ("  [%s]" % _lim_txt(mask)) if mask else ""))
            joint_rows.append(row)

        tip_ok = up_mm < args.tip_mm_max and dn_mm < args.tip_mm_max
        if scale != 1.0:
            invalid.append("行程缩放到 %.0f%%, 不是满行程" % (scale * 100.0))
        if invalid:
            verdict = INVALID
        elif tip_ok and not lag_bad:
            verdict = PASS
        else:
            verdict = FAIL

        reason = "; ".join(invalid) if invalid else "; ".join(
            ([] if tip_ok else ["指尖静差 ↑%.2f ↓%.2fmm 超 %.2fmm"
                                % (up_mm, dn_mm, args.tip_mm_max)])
            + (["延迟超标: " + ", ".join(lag_bad)] if lag_bad else []))
        finger_rows.append({
            "finger": f, "name": FINGER_NAMES[f],
            "tipUpMm": round(up_mm, 3), "tipDnMm": round(dn_mm, 3),
            "maxLagMs": round(max_lag, 2), "verdict": verdict, "reason": reason,
        })
        runner.log("  %s: 最大延迟 %.1fms, 指尖静差 ↑%.2fmm ↓%.2fmm → **%s**%s"
                   % (FINGER_NAMES_CN[f], max_lag, up_mm, dn_mm, verdict,
                      ("  (%s)" % reason) if reason else ""))

        # ── 回零 + 断本指 ───────────────────────────────────────────────
        if not runner.ramp_phase(prof, zeros, RAMP_S, "  回零"):
            aborted = True
        runner.unscope(prof)
        if aborted:
            break

    # ── 本手结论 ────────────────────────────────────────────────────────
    verdicts = [r["verdict"] for r in finger_rows]
    if aborted:
        overall = INVALID
        overall_reason = "中途停机, 只测完 %d 指" % len(finger_rows)
    elif not verdicts:
        overall = INVALID
        overall_reason = "一根手指都没测到"
    elif INVALID in verdicts:
        overall = INVALID
        overall_reason = "判不了: " + ", ".join(
            "%s(%s)" % (FINGER_NAMES_CN[r["finger"]], r["reason"])
            for r in finger_rows if r["verdict"] == INVALID)
    elif FAIL in verdicts:
        overall = FAIL
        overall_reason = "不合格: " + ", ".join(
            "%s(%s)" % (FINGER_NAMES_CN[r["finger"]], r["reason"])
            for r in finger_rows if r["verdict"] == FAIL)
    else:
        overall = PASS
        overall_reason = ""

    runner.log("位控性能QC 结论: **%s**%s"
               % (overall, ("  —— " + overall_reason) if overall_reason else ""))
    with _RESULTS_LOCK:
        _RESULTS[runner.serial_number] = {
            "serial_number": runner.serial_number,
            "handedness": str(runner.handedness),
            "address": runner.address,
            "overallPass": overall == PASS,
            "verdict": overall,
            "reason": overall_reason,
            "gates": {"lagMsMax": args.lag_ms_max, "tipMmMax": args.tip_mm_max,
                      "kp": prof.kp, "kd": prof.kd, "iq": prof.iq,
                      "ki": args.ki, "iclamp": args.iclamp,
                      "freqHz": freq, "margin": args.margin,
                      "ampScale": scale, "cycles": args.cycles},
            "limitsTable": runner.limits.path,
            "fingers": finger_rows,
            "joints": joint_rows,
        }


def _finger_secs(freq: float, args) -> float:
    """每指大致耗时 —— 只为在开头给操作员一个预期, 不参与控制。"""
    lag = 1.0 / freq + LAG_SETTLE_S + args.cycles / freq
    per_axis = 2.0 * (RAMP_S + STAT_SET_S + STAT_MEAS_S)
    return RAMP_S + lag + 4.0 * per_axis + RAMP_S


# ──────────────────────────────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    ap = build_argparser(__doc__.strip().splitlines()[0], examples="""用法:
  python %(prog)s --dry-run                  先看每轴行程和限位余量, 不使能不运动
  python %(prog)s                            正式跑(全部在线手指), 约 3~5 分钟
  python %(prog)s --finger index             只测食指
  python %(prog)s --out qc.json              结果写 JSON, 给自动化设备取数
  python %(prog)s --lag-ms-max 37            按「扣掉 bridge 偏移」的严门限判

退出码: 0 全合格 / 1 有不合格 / 2 连接或参数错误 / 3 跑完了但判不了(见日志)

**这是判合格的工序, 不是老化。** --duration 在这里只当超时兜底, 不是运行时长;
--export-limits 本脚本不支持(要标定限位请用 single_joint_aging.py)。
""")
    ap.add_argument("--finger", metavar="LIST",
                    help="只测指定手指 (默认测全部在线手指)。逗号分隔, "
                         "thumb/index/middle/ring/pinky")
    ap.add_argument("--freq", type=float, default=FREQ_HZ, metavar="HZ",
                    help="锁相正弦频率 (默认 %(default)s, 同上位机)")
    ap.add_argument("--cycles", type=int, default=LAG_CYCLES, metavar="N",
                    help="锁相积分的周期数 (默认 %(default)s, 同上位机)。加大更抗噪, 也更慢")
    ap.add_argument("--margin", type=float, default=MARGIN, metavar="K",
                    help="行程取半程的比例 (默认 %(default)s, 同上位机)")
    ap.add_argument("--ki", type=float, default=KI,
                    help="软件积分增益 (默认 %(default)s, 同上位机)。固件没有 ki")
    ap.add_argument("--iclamp", type=float, default=ICLAMP, metavar="A",
                    help="软件积分的力矩钳位 (默认 %(default)s A, 同上位机)")
    ap.add_argument("--lag-ms-max", type=float, default=LAG_MS_MAX, metavar="MS",
                    help="判据①跟踪延迟上限 (默认 %(default)s = 上位机那个数, 内含约 "
                         "8ms 的 Qt bridge 偏移而 SDK 路径没有 → 实际松约 8ms; "
                         "要一样严用 37)")
    ap.add_argument("--tip-mm-max", type=float, default=TIP_MM_MAX, metavar="MM",
                    help="判据②指尖静差上限 (默认 %(default)s mm = 整手指尖精度指标)")
    ap.add_argument("--out", metavar="FILE",
                    help="把逐关节/逐指结果写成 JSON (一拖多时所有手写在一个文件里)。"
                         "字段名对齐上位机 PosQcJointResult / PosQcFingerResult, "
                         "自动化设备直接取数")
    args = ap.parse_args(argv)

    if args.export_limits:
        print("!! 本脚本不支持 --export-limits —— 它靠「被限位夹住」反推真限位, 而位控QC "
              "一旦夹住就该判「判不了」, 两件事冲突。\n"
              "   要标定这只手的真限位, 用: python single_joint_aging.py "
              "--amp-scale 1.0 --duration 75s --export-limits")
        return 2
    if args.cycles < 1:
        print("!! --cycles 至少 1")
        return 2

    rc = run_multi(build_profile, args,
                   banner="===== WH120 位控性能QC (kp=%.1f kd=%.2f iq=%.1fA, "
                          "延迟 < %.0fms, 指尖 < %.2fmm) ====="
                          % (KP, KD, IQ, args.lag_ms_max, args.tip_mm_max))
    if args.dry_run:
        return rc

    with _RESULTS_LOCK:
        results = list(_RESULTS.values())
    if args.out and results:
        doc = {"_meta": {"tool": "pos_perf_qc.py",
                         "aligned_with": "sboard_qt PosPerfQcWorker",
                         "process": "位控性能QC"},
               "hands": results}
        try:
            with open(args.out, "w", encoding="utf-8") as fp:
                json.dump(doc, fp, ensure_ascii=False, indent=2)
            print("结果已写 %s (%d 只手)" % (args.out, len(results)))
        except OSError as exc:
            print("!! 结果写入失败: %s" % exc)

    print("\n===== 位控性能QC 汇总 =====")
    for r in results:
        print("  %-20s %-6s %s" % (r["serial_number"], r["verdict"], r["reason"]))
    if rc != 0:
        return 2                              # 连接/运行出错, 结论不可信
    if not results:
        return 2
    if any(r["verdict"] == INVALID for r in results):
        return 3
    if any(r["verdict"] == FAIL for r in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
