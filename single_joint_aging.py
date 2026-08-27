#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单关节老化 —— 对齐上位机 `SweepWorker::doStartSingleJointAging`。

**动作**: 每个关节在自己的**全行程**内做正弦往复, **0.5 Hz**, 无限循环。

    pos = center + amp · env · sin(ωt + φ)
    center = (min + max) / 2          ← 不对称限位 → 中心不是 0
    amp    = (max - min) / 2 - 5°     ← 两端各留 5° 安全余量

**原始用途**: 让单个关节反复走到行程极限, 观察相邻轴的编码器是否受磁场干扰。
上位机是在界面上勾选关节跑的; 本 demo **默认跑所有在线关节**, 不用勾选,
`--joints` 可以再筛。

**与空载老化的区别** (两者都是正弦全幅, 但不是一回事):

| | 空载老化 `no_load_aging.py` | 单关节老化 (本脚本) |
|---|---|---|
| 频率 | 0.3 Hz | **0.5 Hz** (更快) |
| 拇指 J2 | 两端非对称内缩 25°/30° | **不缩**, 全行程 |
| 拇指 J3 / 四指 pip | 蜷曲端再缩 10° | **不缩**, 全行程 |
| 四指外展 J2 | 锁成扇形不摆 | **全行程 ±40° 摆** |
| 目的 | 整手长时间空载耐久 | 单轴走极限, 看编码器干扰 |

⚠ **所以整手长跑请用空载老化, 不要用本脚本。** 本脚本少了那几条防撞限幅 ——
上位机之所以能这么跑, 是因为它默认只勾一两个关节。全关节一起跑到极限时,
相邻手指有相互剐蹭的可能, **第一次跑务必先 `--dry-run` 看行程, 再用
`--amp-scale 0.3` 小幅确认不干涉, 然后才放到 1.0**。

防撞仍保留上位机那套「勾」形相位方案(已真机验证): 拇指 J1/J2 相位 π;
四指 J3/J4 (pip/dip) 相位 π 与 J1 (mcp_flex) 反相 —— J1 蜷时 J3/J4 伸,
指尖被推到远端不戳掌心, 相邻指尖错位。
"""

from __future__ import annotations

import math
import sys
from typing import List

from aging_common import (
    AGING_MARGIN_RAD, PI, TOTAL_JOINTS,
    HandRunner, JointLimits, Profile,
    build_argparser, flat_to_nid, joint_label, run_multi, slot_of,
)

# ── 运动参数 (对应上位机 SWEEP_TABLE[2] + 运动参数中心「扫频验证/老化」一行) ──
FREQ_HZ = 0.5                  # SWEEP_TABLE[2].freq —— 单关节不慢跑, 保持 0.5Hz
CYCLES = 2                     # SWEEP_TABLE[2].cycles, 只用来定振幅渐入时长
KP, KD, IQ = 5.0, 0.1, 1.0     # DEFS[SweepAging] = {5.0, 0.10, 1.0}
RAMP_FRACTION = 0.25


def build_profile(ctx: HandRunner) -> Profile:
    limits: JointLimits = ctx.limits

    center = [0.0] * TOTAL_JOINTS
    amp = [0.0] * TOTAL_JOINTS
    phase = [0.0] * TOTAL_JOINTS
    enabled = [False] * TOTAL_JOINTS

    # 默认跑所有在线关节; --joints 可以再筛一遍(上位机的「勾选」等价物)。
    wanted = ctx.online_idxs()
    if ctx.args.joints:
        picked = parse_joint_filter(ctx.args.joints)
        wanted = [k for k in wanted if k in picked]
        if not wanted:
            raise ValueError("--joints 筛完一个在线关节都不剩")

    for k in wanted:
        ca = limits.center_amp(k, AGING_MARGIN_RAD)
        if ca is None:                      # 限位表里没有 → 不敢跑全幅, 跳过
            continue
        center[k], amp[k] = ca
        enabled[k] = True
        # 「勾」形防撞相位, 与上位机一致。
        nid = flat_to_nid(k)
        if nid in (1, 2) or (nid >= 5 and slot_of(k) in (2, 3)):
            phase[k] = PI

    if not any(enabled):
        raise RuntimeError("没有可跑的在线关节")

    omega = 2.0 * PI * FREQ_HZ

    def motion(t: float, env: float):
        pos = [0.0] * TOTAL_JOINTS
        vel = [0.0] * TOTAL_JOINTS
        for j in JOINT_RANGE:
            a = amp[j]
            if a == 0.0:
                pos[j] = center[j]
                continue
            th = omega * t + phase[j]
            pos[j] = center[j] + a * env * math.sin(th)
            vel[j] = a * env * omega * math.cos(th)
        return pos, vel

    idxs = [k for k in JOINT_RANGE if enabled[k]]
    offline = [k for k in JOINT_RANGE if not ctx.online[k]]
    notes = ["跑在线关节 %d 个: %s" % (len(idxs), ", ".join(joint_label(k) for k in idxs))]
    if offline:
        notes.append("离线 %d 个不参与: %s"
                     % (len(offline), ", ".join(joint_label(k) for k in offline)))
    notes.append("全行程正弦 %.2fHz —— 比空载老化少几条防撞限幅(拇指 J2/J3、四指 pip、"
                 "四指外展都是全行程), 首次整手跑先 --amp-scale 0.3 确认不剐蹭。" % FREQ_HZ)

    return Profile(
        name="单关节老化 (全行程 %.2fHz)" % FREQ_HZ,
        kp=KP, kd=KD, iq=IQ,
        center=center,
        motion=motion,
        enabled=enabled,
        ramp_s=(CYCLES / FREQ_HZ) * RAMP_FRACTION,
        stop_style="sine",
        feedforward=True,
        period_s=1.0 / FREQ_HZ,
        notes=notes,
    )


JOINT_RANGE = range(TOTAL_JOINTS)


def parse_joint_filter(text: str) -> List[int]:
    """'index' / 'index_J1' / '6' (flat 下标) / 'nid:8' 混着写, 逗号分隔。"""
    from aging_common import FINGER_NAMES, finger_index, nid_to_flat
    picked: List[int] = []
    for tok in (t.strip() for t in text.split(",") if t.strip()):
        low = tok.lower()
        if low.startswith("nid:"):
            k = nid_to_flat(int(low[4:]))
            if k < 0:
                raise ValueError("NID %s 不是电机关节" % low[4:])
            picked.append(k)
        elif "_j" in low:                            # index_J3
            name, j = low.split("_j", 1)
            picked.append(finger_index(name) * 4 + int(j) - 1)
        elif low in FINGER_NAMES:                    # 整根手指
            picked.extend(range(finger_index(low) * 4, finger_index(low) * 4 + 4))
        elif low.isdigit():                          # flat 下标
            picked.append(int(low))
        else:
            raise ValueError("看不懂的关节写法: %r" % tok)
    bad = [k for k in picked if not 0 <= k < TOTAL_JOINTS]
    if bad:
        raise ValueError("关节下标越界: %s" % bad)
    return picked


def main(argv=None) -> int:
    ap = build_argparser(__doc__.strip().splitlines()[0], examples="""用法:
  python %(prog)s --dry-run                    先看全行程表(本脚本行程比空载老化大)
  python %(prog)s --amp-scale 0.3              整手跑之前**务必**先小幅确认不剐蹭
  python %(prog)s --joints index_J3            只跑食指 J3
  python %(prog)s --joints thumb,index_J3      拇指整根 + 食指 J3

--joints 写法: index(整根) / index_J3(单轴) / nid:8(固件 NID) / 6(flat 下标), 逗号分隔。
不给 --joints 就跑**全部在线关节**, 不用勾选。
""")
    ap.add_argument("--joints", metavar="LIST",
                    help="只跑指定关节 (默认跑全部在线关节)。逗号分隔, 支持 "
                         "`index` / `index_J3` / `nid:8` / flat 下标 `6`")
    args = ap.parse_args(argv)
    return run_multi(build_profile, args,
                     banner="===== WH120 单关节老化 demo (全行程 %.2fHz) =====" % FREQ_HZ)


if __name__ == "__main__":
    sys.exit(main())
