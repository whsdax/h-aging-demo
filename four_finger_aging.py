#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整指老化·四指专用 —— 治具 5 个槽位全挂四指模组(食/中/无名/小), 不管插在哪个口。

**跟 `single_finger_aging.py` 是同一道工序**(17 段梯形, J1/J3/J4 ±45° →
J1 屈 90° 保持后 J2 外展 ±35°, 同样的 kp/kd/iq 和停机缓释放), 差别只有一处:
**全部 5 个槽位都按四指限位钳位**, 不再按「插在哪个口」认指型。

为什么这样就够: 食指/中指/无名指/小指的设计限位**完全一致**(J1 -60~+90°,
J2 ±40°, J3 -60~+120°, J4 -60~+90°), 所以四指之间互插本来就无需区分。唯一要处理
的是槽 0 —— 那是语义拇指口, 默认会按拇指限位钳(J1 上限 74°, J2 下限 -85°), 四指
插上去行程会偏小。本脚本把四指限位套到所有槽位, 操作员随便插。

**四指的名义行程一条都不会被削** —— 启动日志里不该出现「限位削幅」。出现了就说明
限位表不对(比如显式 --limits 传了别的表), 停下来查。

拇指模组请用 `thumb_aging.py`, 别用本脚本(会被按四指的 J1 +90° / J2 -40° 下发,
拇指 J1 只到 +74°, 每圈顶死限位)。
"""

from __future__ import annotations

import sys

from aging_common import finger_index
import finger_aging_core as core

EXAMPLES = """用法:
  python %(prog)s --dry-run                 先看限位和行程, 不使能不运动
  python %(prog)s                           跑全部在线槽位(治具满载就是 5 个四指模组)
  python %(prog)s --slot 1,2,3,4            跳过槽 0(拇指口)
  python %(prog)s --duration 8h             跑 8 小时后自动安全停机

槽位号 = 脊髓板第几路 (0 = 语义拇指口)。本脚本不关心插在哪个口, 一律按四指限位跑。
食/中/无名/小 的限位完全一致, 所以这几种模组之间也不用区分。

挂负重块时: 停机会先回零, 再把扭矩上限 2 秒内缓降到 0A 才断使能。
**别强杀进程**, 否则手指自由落体砸下去。
"""


def main(argv=None) -> int:
    # 参考指位取食指 —— 四指的设计限位逐条相同, 取哪一根都一样。
    return core.run(
        ref_finger=finger_index("index"), kind_cn="四指",
        prog_doc=__doc__.strip().splitlines()[0], examples=EXAMPLES, argv=argv)


if __name__ == "__main__":
    sys.exit(main())
