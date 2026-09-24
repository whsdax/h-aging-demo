#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整指老化 —— 「5 槽位同型手指」治具的共用核心。

`thumb_aging.py`(专跑拇指) 和 `four_finger_aging.py`(专跑四指) 都是本模块的薄壳,
两者只差一个**参考指位**。

**为什么要分成两个脚本**: 指位身份只由「插在脊髓板哪个口」决定 —— 关节板 flash 里
只有 `ImmutableMeta.device_addr`(总线内第几个电机), 不上报自己是哪根手指。所以治具
上 5 个槽位挂 5 个拇指时, 脚本按 NID 查表拿到的是 thumb/index/.../pinky 五套不同
限位, 只有槽 0 那套是对的。拇指与四指的限位差得不少:

    轴   拇指              四指(四个槽完全一致)
    J1   -68 ~ **+74°**    -60 ~ **+90°**
    J2   **-85** ~ +40°    -40 ~ +40°
    J3   -60 ~ +90°        -60 ~ **+120°**
    J4   -60 ~ +90°        -60 ~ +90°

按错的表钳位: 四指当拇指跑只是行程偏小(J1 少走 16°), 拇指当四指跑则会每圈往 +90°
顶死(固件 20kHz 环里有 `final_pos` 兜底 clamp, 机械打不坏, 但按 IQ 顶十几度、
老化数据作废)。所以**一台电脑一种指型**, 由脚本把参考指位的限位套到全部槽位上
(`FingerRemappedLimits`), 操作员不用关心插在第几个口。

**波形与增益整套复用 `single_finger_aging.py`** —— 要跑的是同一道工序(17 段梯形,
J1/J3/J4 ±45° → J1 屈 90° 保持后 J2 外展 ±35°), 只是治具从「单指」变成「5 槽位
同型指」。从那里 import 而不是抄一份, 是为了以后调波形时两边不会走偏。

**与 single_finger_aging.py 的三处差别**:
  1. 限位查表统一到参考指位(上面说的)。
  2. 默认跑**所有在线槽位**(治具本来就是 5 个一起跑), `--slot` 按槽位号筛。
  3. 不默认读 `limits_<SN>.json` —— 那张表是**按槽位**标定的, 而这里 5 个槽位的
     模组随时在换, 按槽位记的实测值对不上下一批模组。要用就显式 `--limits`。

**没有「相邻指剐蹭」这回事**: 治具上是 5 个互相独立的槽位, 不是一只装配好的手,
所以不像 single_finger_aging.py 那样在多指同跑时告警。
"""

from __future__ import annotations

from typing import Callable, List, Tuple

from aging_common import (
    R2D, TOTAL_JOINTS,
    FINGER_NAMES, FingerRemappedLimits, HandRunner, JointLimits, Profile,
    build_argparser, finger_of, joint_label, run_multi, slot_of, smoothstep,
)
# 同一道工序 —— 波形、增益、停机缓释放全部取自单指负载老化, 别在这里另立一套。
from single_finger_aging import (
    CYCLE_S, IQ, KD, KP, RELEASE_FLOOR_A, RELEASE_S, RELEASE_STEPS, SEGMENT_S, STEPS,
)

#: 槽位数 = 脊髓板的 5 路总线 (语义指位序: 0 = 拇指口)
SLOT_COUNT = 5


def slot_label(k: int) -> str:
    """按**槽位**而不是指名描述一个关节 —— 操作员是按槽位号拔插模组的。"""
    return "槽%d_J%d" % (finger_of(k), slot_of(k) + 1)


def parse_slots(text: str) -> List[int]:
    """'0,2,4' → [0, 2, 4]。越界当场报错, 别等到筛完发现一个都不剩。"""
    out: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            n = int(part)
        except ValueError:
            raise ValueError("--slot 只认槽位号 0..%d, 不认 %r" % (SLOT_COUNT - 1, part))
        if not 0 <= n < SLOT_COUNT:
            raise ValueError("槽位号 %d 越界 (只有 0..%d)" % (n, SLOT_COUNT - 1))
        if n not in out:
            out.append(n)
    return sorted(out)


def make_build_profile(ref_finger: int, kind_cn: str) -> Callable[[HandRunner], Profile]:
    """生成一个 `build_profile` —— `ref_finger` 是被测模组的真实指位。"""

    def build_profile(ctx: HandRunner) -> Profile:
        args = ctx.args

        # ① 限位统一到被测指型。**必须在读 ctx.limits 之前替换掉** —— 替换后
        #    describe_profile()/--export-limits 看到的也是这一份(HandRunner.run()
        #    里 build_profile 先于它们执行)。
        if getattr(args, "per_slot_limits", False):
            limits = ctx.limits
            limit_note = ("限位: 按槽位原样查表(--per-slot-limits) —— 只在限位表本来"
                          "就是这套治具导出的时候才对。")
        else:
            limits = FingerRemappedLimits(ctx.limits, ref_finger)
            ctx.limits = limits
            limit_note = ("限位: 全部 %d 个槽位统一用 **%s** 的限位 (%s) —— "
                          "指位身份只由插在哪个口决定, 模组自己不上报指型。"
                          % (SLOT_COUNT, kind_cn, FINGER_NAMES[ref_finger]))

        # ② 选槽位: 默认所有在线的(治具就是 5 个一起跑), --slot 再筛。
        online = ctx.online_idxs()
        slots = sorted({finger_of(k) for k in online})
        if not slots:
            raise RuntimeError("没有在线的槽位 —— 检查模组有没有插到位、有没有上电")
        if getattr(args, "slot", None):
            picked = parse_slots(args.slot)
            slots = [s for s in slots if s in picked]
            if not slots:
                raise ValueError("--slot 选的槽位 %s 一个都不在线"
                                 % ",".join(str(s) for s in picked))

        # ③ 只使能这些槽位里**真正在线**的轴 —— 某个槽位可能不是四轴全在。
        enabled = [False] * TOTAL_JOINTS
        for k in online:
            if finger_of(k) in slots:
                enabled[k] = True

        # ④ 每段的 (起点, 终点, 起始时刻, 时长)。起点 = 上一段的终点; 第 0 段的
        #    上一段是本圈最后一段, 其终点是全零, 与回零斜坡末尾连续。
        bounds: List[Tuple[Tuple[float, ...], Tuple[float, ...], float, float]] = []
        t_acc = 0.0
        prev = STEPS[-1][0]
        for target, dur, _label in STEPS:
            bounds.append((prev, target, t_acc, dur))
            t_acc += dur
            prev = target

        # 名义角 → 各槽位各轴的钳位后目标。钳位在下发这一步做, 插值仍在名义空间算。
        # 这里 5 个槽位的限位已经统一, 所以钳出来的值也一样 —— 但保留逐轴 clamp,
        # 因为 --per-slot-limits 下它们会不同。
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
            for f in slots:
                for j in range(4):
                    k = f * 4 + j
                    pos[k] = limits.clamp(k, nominal[j])
            return pos, [0.0] * TOTAL_JOINTS

        notes = [
            limit_note,
            "槽位: %s (在线 %d 轴), 一圈 %.1fs"
            % (", ".join("槽%d" % s for s in slots),
               sum(1 for e in enabled if e), CYCLE_S),
            "日志里的 %s 等前缀是**槽位名**(槽 N = 脊髓板第 N 路), 不是被测模组的指型 —— "
            "本场跑的全是%s。" % (joint_label(0), kind_cn),
            "停机会先回零, 再把扭矩上限 %.2fA → %.2fA 缓降 %.1fs, 最后断使能 —— "
            "挂负重块时别强杀进程, 否则手指自由落体。" % (IQ, RELEASE_FLOOR_A, RELEASE_S),
        ]

        # 提前把「哪些轴的名义行程会被限位削掉」算出来 —— 拇指场是预期会削的
        # (J1 名义 +90°, cmc_flex 上限只有 +74°), 不提示的话看着像动作没做全。
        # 限位统一时 5 个槽位削出来一模一样, 报一次就够; --per-slot-limits 下才逐槽报。
        report_slots = slots if getattr(args, "per_slot_limits", False) else slots[:1]
        for f in report_slots:
            clipped = []
            for j in range(4):
                k = f * 4 + j
                want_lo = min(step[0][j] for step in STEPS)
                want_hi = max(step[0][j] for step in STEPS)
                got_lo, got_hi = limits.clamp(k, want_lo), limits.clamp(k, want_hi)
                if abs(got_lo - want_lo) > 1e-6 or abs(got_hi - want_hi) > 1e-6:
                    clipped.append("J%d %.0f~%.0f° → %.0f~%.0f°"
                                   % (j + 1, want_lo * R2D, want_hi * R2D,
                                      got_lo * R2D, got_hi * R2D))
            if not clipped:
                continue
            who = ("槽%d" % f) if len(report_slots) > 1 else "每个槽位都一样"
            notes.append("限位削幅(%s, %s): %s —— 这是预期的, 不是动作没做全。"
                         % (who, kind_cn, "; ".join(clipped)))

        return Profile(
            name="整指老化·%s (%d 槽位)" % (kind_cn, len(slots)),
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

    return build_profile


def run(ref_finger: int, kind_cn: str, prog_doc: str, examples: str, argv=None) -> int:
    """两个薄壳脚本共用的 main。"""
    ap = build_argparser(prog_doc, examples=examples)
    ap.add_argument("--slot", metavar="LIST",
                    help="只跑指定槽位 (默认跑全部在线槽位)。逗号分隔的槽位号 0..4, "
                         "0 = 脊髓板语义拇指口")
    ap.add_argument("--per-slot-limits", action="store_true",
                    help="不做指型统一, 按槽位原样查限位表。只在限位表本来就是这套"
                         "治具导出的时候用")
    # 共用骨架的 --limits 帮助说「默认按 SN 自动找 limits_<SN>.json」, 这两个脚本
    # 故意不那么做(见下面钉设计表的说明), 帮助文字也得改掉, 否则 --help 就是错的。
    for act in ap._actions:
        if "--limits" in getattr(act, "option_strings", ()):
            act.help = ("强制指定关节限位表 JSON。**一般不用写** —— 本脚本默认钉在设计表 "
                        "joint_limits.json 上, 不去找 limits_<SN>.json: 那张表是按**槽位**"
                        "标定的, 而这套治具上的模组随时在换, 按槽位记的实测值对不上下一批模组")
    args = ap.parse_args(argv)

    # 默认钉在**设计表**上, 不自动去找 limits_<SN>.json: 那张表是按槽位标定的,
    # 而这套治具上的模组随时在换, 按槽位记的实测值对不上下一批模组。
    # (显式 --limits 仍然照办 —— args.limits 非空时 HandRunner._select_limits 直接返回。)
    if not args.limits:
        try:
            args.limits = JointLimits.resolve(None)
        except FileNotFoundError as exc:
            print("!! %s" % exc)
            return 2

    return run_multi(
        make_build_profile(ref_finger, kind_cn), args,
        banner="===== WH120 整指老化·%s (一圈 %.1fs, %d 槽位同型) ====="
               % (kind_cn, CYCLE_S, SLOT_COUNT))
