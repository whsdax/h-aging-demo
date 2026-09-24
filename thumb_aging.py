#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整指老化·拇指专用 —— 治具 5 个槽位全挂拇指模组, 不管插在哪个口。

**跟 `single_finger_aging.py` 是同一道工序**(17 段梯形, J1/J3/J4 ±45° →
J1 屈 90° 保持后 J2 外展 ±35°, 同样的 kp/kd/iq 和停机缓释放), 差别只有一处:
**全部 5 个槽位都按拇指限位钳位**, 不再按「插在哪个口」认指型。

为什么要这一处差别: 关节板 flash 里只有总线内槽号, 不上报自己是哪根手指 —— 指位
身份完全由插在脊髓板哪个口决定。所以拇指插到槽 1..4 时, 默认会被按四指限位钳,
J1 会往 +90° 顶(拇指 cmc_flex 上限只有 +74°), 每圈堵转顶死机械限位。本脚本把拇指
限位套到所有槽位, 操作员随便插。

**预期会看到「限位削幅 J1 0~90° → 0~74°」** —— 序列的名义角是拇指/四指共用的,
拇指 J1 天然走不到 90°。这是正常的, 不是动作没做全。

四指模组请用 `four_finger_aging.py`, 别用本脚本(会被按拇指的 J2 -85° 甩到
四指走不到的角度; 固件有兜底 clamp 打不坏, 但会堵转)。
"""

from __future__ import annotations

import sys

from aging_common import finger_index
import finger_aging_core as core

EXAMPLES = """用法:
  python %(prog)s --dry-run                 先看限位和行程, 不使能不运动
  python %(prog)s                           跑全部在线槽位(治具满载就是 5 个拇指)
  python %(prog)s --slot 0,1                只跑槽 0 和槽 1
  python %(prog)s --duration 8h             跑 8 小时后自动安全停机

槽位号 = 脊髓板第几路 (0 = 语义拇指口)。本脚本不关心插在哪个口, 一律按拇指限位跑。

挂负重块时: 停机会先回零, 再把扭矩上限 2 秒内缓降到 0A 才断使能。
**别强杀进程**, 否则手指自由落体砸下去。
"""


def main(argv=None) -> int:
    return core.run(
        ref_finger=finger_index("thumb"), kind_cn="拇指",
        prog_doc=__doc__.strip().splitlines()[0], examples=EXAMPLES, argv=argv)


if __name__ == "__main__":
    sys.exit(main())
