#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WH120 老化 demo —— 公共骨架。

四个老化脚本只负责「算轨迹」; 连接 / 使能 / 回零 / 200Hz 下发 / 安全停机这些
**危险且容易写错**的部分全在本文件。二开时只改轨迹, 不用碰这一层。

一拖多: 每只手一个 `HandRunner` 线程, 各自独立连接 + 独立 200Hz 命令流。

========================  四条安全纪律 (踩了会炸手)  ========================
1. **enable 前必须先把 `actual_position` 当目标广播出去**。固件 MIT 的 `theta_des`
   上电默认 0, 不预置就使能 = 20 个关节同时从当前姿态弹向 0 → 母线电流尖峰 →
   欠压 → 整手掉线。见 `_preseed_and_enable()`, 别删。
2. **位置种子取 `joint_states().position`(关节侧角)**, 不能用电机侧多圈角度(会甩飞)。
3. **未选中的 slot 填「保位值」(使能瞬间的实际角), 不能填 0** —— 填 0 会把那些
   关节拉向零点。见 `_emit()`。
4. **任何退出路径(含 Ctrl+C / 异常)都必须回零 + disable**。老化循环是无限的。
   见 `_shutdown()`。

波形与增益逐条对齐无极内部上位机 sboard_qt; 差异清单见 README「要知道的几件事」。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import wuji_sdk
from wuji_sdk import ConnectOptions, JointCommand, SdkManager

# ──────────────────────────────────────────────────────────────────────────
# 常量
# ──────────────────────────────────────────────────────────────────────────

PI = math.pi
D2R = PI / 180.0
R2D = 180.0 / PI

TOTAL_JOINTS = 20
JOINTS = range(TOTAL_JOINTS)
PUB_RATE_HZ = 200
TICK_S = 1.0 / PUB_RATE_HZ
LATE_RESYNC_S = 0.05        # 节拍被系统拖过这么久就重新对齐, 别让 sleep 目标越积越旧
STATUS_INTERVAL_S = 60.0    # 老化中每隔这么久打一行进度(已跑多久 / 母线 / 电流 / 温度 / 故障)

# 状态字里的三个限幅标志, 自己压成一个掩码方便累积。踩到限流 = 堵转或撞上了。
LIM_POS, LIM_VEL, LIM_CUR = 1, 2, 4

# 一拖多的起跑阶段串行锁 —— 见 HandRunner.run() 里的说明。
_STARTUP_LOCK = threading.Lock()

FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
FINGER_NAMES_CN = ("拇指", "食指", "中指", "无名指", "小指")

# —— 以下逐个对应上位机 SweepWorker.h 里的同名常量 ——
KD_FF = 0.5                          # 速度前馈增益 → 固件 tau_ff_jnt, 克服静摩擦
ZERO_DURATION_S = 3.0                # 启动回零/就位斜坡时长
ABORT_RAMPDOWN_S = 1.0               # 停止① 沿原正弦轨迹把摆幅收到 0
ABORT_HOLD_S = 0.5                   # 停止② 停稳保持
ABORT_RETURN_S = 2.0                 # 停止③ 柔顺回 0
ZERO_KP, ZERO_KD = 3.0, 0.05         # 回零段增益
SETTLE_KP, SETTLE_KD = 12.0, 0.3     # 停止④ 贴零段增益(收 1~2° 残差)
SETTLE_S = 1.0                       # 贴零段超时兜底
SETTLE_TOL_RAD = 0.6 * D2R           # 贴零「到位即松」阈值
AGING_MARGIN_RAD = 5.0 * D2R         # 老化行程在限位两端各内缩的安全余量

PRESEED_FRAMES = 20                  # enable 前预置保位帧数 (20 × 5ms = 100ms)
POST_ENABLE_FRAMES = 10              # enable 后继续保位的帧数
FRAME_INTERVAL_S = 0.005

DEFAULT_LIMITS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "joint_limits.json")

_ZEROS = [0.0] * TOTAL_JOINTS


# ──────────────────────────────────────────────────────────────────────────
# 关节编号映射
#
#   flat 下标 k : 0..19, finger-major。SDK 的 `hand.joint(k)` / enable 掩码 /
#                 joint_command 列表都用它。k//4 = 手指, k%4 = 该指第几轴。
#   固件 NID    : 1..24, 每指占 5 个号(第 5 个是触觉节点, 无电机)。
#                 `joint_states()` 回来的 `nid` 用的是它。
# ──────────────────────────────────────────────────────────────────────────

def flat_to_nid(k: int) -> int:
    return (k // 4) * 5 + (k % 4) + 1


def nid_to_flat(nid: int) -> int:
    """触觉节点 / 非法值返回 -1。"""
    bus, slot = divmod(nid - 1, 5)
    return -1 if (nid < 1 or nid > 25 or slot >= 4) else bus * 4 + slot


def finger_of(k: int) -> int:
    return k // 4                       # 0=拇指 .. 4=小指


def slot_of(k: int) -> int:
    return k % 4                        # 0=J1 .. 3=J4


def joint_label(k: int) -> str:
    return "%s_J%d" % (FINGER_NAMES[finger_of(k)], slot_of(k) + 1)


def finger_index(name: str) -> int:
    """手指名 → 手指号; 中英文都认。"""
    if name.strip().lower() in FINGER_NAMES:
        return FINGER_NAMES.index(name.strip().lower())
    if name.strip() in FINGER_NAMES_CN:
        return FINGER_NAMES_CN.index(name.strip())
    raise ValueError("未知手指名 %r (可用: %s)" % (name, ", ".join(FINGER_NAMES)))


def smoothstep(u: float) -> float:
    """3u²-2u³ —— 两端速度为 0 的 S 曲线。"""
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


# ──────────────────────────────────────────────────────────────────────────
# 关节限位表
# ──────────────────────────────────────────────────────────────────────────

class JointLimits:
    """按 flat 下标查询的关节机械范围。

    ⚠ 数据是**设计限位**, 不是该手的实测限位: 公开 SDK 没暴露固件的
    `ps_final_pos_*`。要贴合具体某只手, 改一份 JSON 用 --limits 传进来。
    """

    #: 逐手实测表的命名约定 —— --export-limits 按这个写, 加载时按这个找
    PER_HAND_FMT = "limits_%s.json"

    def __init__(self, path: Optional[str] = None):
        self.path = self.resolve(path)
        with open(self.path, encoding="utf-8") as fp:
            doc = json.load(fp)
        ranges = doc["ranges"]
        # 表是哪只手的。设计表没有这个字段 → 空串 = 通用表, 不做 SN 校验。
        self.serial_number = str(doc.get("_meta", {}).get("serial_number", ""))
        self._rng: Dict[int, Tuple[float, float, str]] = {}
        self._src: Dict[int, str] = {}      # 该关节的限位是哪来的(--export-limits 迭代时要传下去)
        for nid_str, v in ranges.items():
            k = nid_to_flat(int(nid_str))
            lo, hi = float(v["min_rad"]), float(v["max_rad"])
            if k >= 0 and math.isfinite(lo) and math.isfinite(hi) and lo < hi:
                self._rng[k] = (lo, hi, v.get("name", ""))
                self._src[k] = str(v.get("source", ""))

    @staticmethod
    def resolve(path: Optional[str]) -> str:
        """定位限位表: 显式指定 → 当前目录 → 脚本目录。

        找两处是为了让各工位能在自己的目录放一份改过的表, 不用每次写 --limits。
        """
        if path:
            if not os.path.isfile(path):
                raise FileNotFoundError("限位表不存在: %s" % path)
            return path
        tried = [os.path.join(os.getcwd(), "joint_limits.json"), DEFAULT_LIMITS_FILE]
        for c in tried:
            if os.path.isfile(c):
                return c
        raise FileNotFoundError("找不到 joint_limits.json, 已尝试:\n  "
                                + "\n  ".join(tried) + "\n用 --limits <文件> 指定。")

    def get(self, k: int) -> Optional[Tuple[float, float]]:
        r = self._rng.get(k)
        return None if r is None else (r[0], r[1])

    @classmethod
    def find_for_sn(cls, sn: str) -> Optional[str]:
        """找这只手自己的实测限位表 `limits_<SN>.json`(当前目录 → 脚本目录)。

        脚本要面对很多只手, 而且随时会来一只没标定过的新手 —— 所以不能靠人去写
        对文件名: 按 SN 自动认, 找不到就退回设计表并告警。
        """
        if not sn:
            return None
        name = cls.PER_HAND_FMT % sn
        for d in (os.getcwd(), os.path.dirname(DEFAULT_LIMITS_FILE)):
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
        return None

    def is_measured(self, k: int) -> bool:
        """这个关节的限位是不是**实测**出来的(而非图纸设计值)。

        兼容两代 source 文案: 新版以「实测」开头, 旧版写成 `min=实测`。
        注意旧版的回退文案是「设计值(未被限位夹住, 无从实测)」—— 也含「实测」二字,
        所以不能用简单的子串包含来判, 否则整表都会被误判成实测。
        """
        src = self._src.get(k, "")
        return src.startswith("实测") or "=实测" in src

    def clamp(self, k: int, rad: float) -> float:
        """钳进该关节限位; 表里没有该关节就原值透传。"""
        r = self._rng.get(k)
        return rad if r is None else min(r[1], max(r[0], rad))

    def center_amp(self, k: int, margin_rad: float = AGING_MARGIN_RAD
                   ) -> Optional[Tuple[float, float]]:
        """全行程正弦的 (中心, 振幅), 两端各内缩 margin。"""
        r = self.get(k)
        if r is None:
            return None
        return (r[0] + r[1]) * 0.5, max(0.0, (r[1] - r[0]) * 0.5 - margin_rad)


class FingerRemappedLimits(JointLimits):
    """把**一个参考指位**的限位复制到全部 5 个槽位。

    为什么需要: 指位身份只由「插在脊髓板哪个口」决定 —— 关节板 flash 里只有
    `ImmutableMeta.device_addr`(总线内第几个电机, 1..5), 不上报自己是哪根手指。
    所以 5 个槽位挂 5 个**同型**手指模组时(专跑拇指 / 专跑四指的治具), 按 NID
    查表会拿到 thumb/index/middle/ring/pinky 五套**不同**限位, 其中只有一套对。
    拇指与四指的差别不小(J1 上限 74° vs 90°, J2 下限 -85° vs -40°), 按错的表
    钳位轻则行程不足, 重则每圈顶死限位。

    本类只改「k → 限位」这一层查表, **不动** flat 下标, 也不动 `joint_label()`:
    日志里仍然打 `index_J1` 这种**槽位名**, 因为操作员是按槽位号去拔插模组的,
    不是按指名。

    注意 `is_measured()` 也跟着取参考槽的来源标记 —— 所以 `--export-limits`
    导出的表拿去给**另一种**指型的治具用会是错的; 这两个专用脚本默认就只读设计表。
    """

    def __init__(self, base: JointLimits, ref_finger: int):
        self.path = base.path
        self.serial_number = base.serial_number
        self.ref_finger = ref_finger
        self._rng: Dict[int, Tuple[float, float, str]] = {}
        self._src: Dict[int, str] = {}
        for k in JOINTS:
            ref_k = ref_finger * 4 + slot_of(k)
            if ref_k in base._rng:
                self._rng[k] = base._rng[ref_k]
                self._src[k] = base._src.get(ref_k, "")


# ──────────────────────────────────────────────────────────────────────────
# 老化档案 —— 每个 demo 脚本产出一个 Profile, 骨架照着跑
# ──────────────────────────────────────────────────────────────────────────

# 轨迹函数: (t, env) -> (positions[20], velocities[20]), 单位 rad / rad·s⁻¹
#   t   从运动段开始起算的秒数, **永远累加不 wrap**(取模会让正弦相位跳变);
#   env 振幅包络 0..1, 骨架用它做起步渐入和停止收摆 —— **只乘振幅, 别乘中心位置**。
# 也可以返回三元组 (pos, vel, eff), 自己接管前馈力矩通道 —— 见 `_emit()`。
MotionFn = Callable[[float, float], Tuple[List[float], List[float]]]

# 编排函数: (runner, profile) -> None。
#
# 给**不是「一个无限循环的波形」**的工序用 —— 典型是位控性能QC: 逐指使能、测一段、
# 算一个数、再换下一指。这种流程装不进 `motion(t, env)`: 它要在阶段之间阻塞着调 SDK
# (切使能范围), 还要拿上一段的测量结果决定下一段。
#
# 非 None 时 `HandRunner.run()` 跑完 `_configure()` 就把控制权交给它, 不走默认的
# 「预置使能 → 回零 → 无限运动循环 → 停机序列」四段链; 使能范围由它自己用
# `runner.scope()` / `runner.unscope()` 逐批切。`_shutdown()` 照常兜底(安全纪律 #4)。
SequenceFn = Callable[["HandRunner", "Profile"], None]


@dataclass
class Profile:
    """一次老化运行的全部参数。"""

    name: str
    kp: float
    kd: float
    iq: float
    center: List[float]              # 启动回零斜坡的终点(= 运动段 t=0 的位置)
    motion: MotionFn
    enabled: List[bool]              # 本次真正驱动/使能的 20 个 slot
    ramp_s: float = 0.0              # 起步的振幅渐入时长(0=不渐入)
    stop_style: str = "sine"         # "sine"=收摆后回零 / "freeze"=冻结目标后回零
    feedforward: bool = True         # 是否下发 velocity + KD_FF 前馈
    freeze_ramp_s: float = 1.6       # stop_style="freeze" 的回零斜坡时长
    release_s: float = 0.0           # >0 → 断使能前做「扭矩上限缓释放」
    release_steps: int = 10
    release_floor_a: float = 0.0
    period_s: float = 10.0           # 动作周期(仅 --dry-run 采样行程用)
    notes: List[str] = field(default_factory=list)   # 启动时打印给操作员的提示
    #: 非 None → 由它接管整个运动流程, 不走默认四段链。见 `SequenceFn`。
    sequence: Optional[SequenceFn] = None
    #: 收尾时整手 disable, 而不是只断 `enabled` 那几个。
    #: `sequence` 会逐批改 `enabled`, 收尾时它只剩最后一批 —— 前面几批就漏掉了。
    disable_all_on_exit: bool = False

    def mask(self) -> List[int]:
        return [int(e) for e in self.enabled]

    def idxs(self) -> List[int]:
        return [k for k, e in enumerate(self.enabled) if e]


# build_profile(ctx) -> Profile; ctx 就是 HandRunner, 提供 .limits/.actual/.handedness/.args
ProfileBuilder = Callable[["HandRunner"], Profile]


# ──────────────────────────────────────────────────────────────────────────
# 单手运行器
# ──────────────────────────────────────────────────────────────────────────

class HandRunner(threading.Thread):
    """一只手的完整老化会话, 跑在自己的线程上。

    连接 → 预置保位(电机未通电) → 使能 → 回零/就位(3s)
         → 运动循环(无限, 直到 stop_event) → 停机序列 → disable → 断开
    """

    def __init__(self, address: str, alias: str, build_profile: ProfileBuilder,
                 stop_event: threading.Event, limits: JointLimits, args):
        super().__init__(name=alias)
        self.address, self.alias = address, alias
        self.build_profile = build_profile
        self.stop_event, self.limits, self.args = stop_event, limits, args

        self.hand = None
        self.serial_number = "?"
        self.handedness = "unknown"
        self.actual: List[float] = [0.0] * TOTAL_JOINTS   # 关节侧实际角(rad)
        self.online: List[bool] = [False] * TOTAL_JOINTS  # 出现在 joint_states 里 = 在线
        self.ok = False
        self.error: Optional[str] = None

        self._sub = self._pub = self._diag_sub = None
        self._diag = None                      # 最新一帧诊断汇总, 见 _on_diag
        self._vbus_min = 99.0
        self._temp_max = 0.0
        self._temp_by_nid: Dict[int, float] = {}   # 逐关节温度峰值
        self._temp_base: Dict[int, float] = {}     # 运动开始时的逐关节温度(算温升)
        self._i_peak = 0.0                     # 整场电流峰值
        self._i_win = 0.0                      # 本状态行窗口内的电流峰值, 每行清零
        # 故障/限幅是**粘性累积**的: 诊断流 ~1kHz 而状态行 60s 一行, 只看最新一帧会
        # 漏掉中间闪现又自清的故障 —— 而那种「闪一下」恰恰是最该抓的早期征兆。
        self._err_seen: Dict[int, int] = {}    # 本轮出现过的故障码 (nid → 码的按位或)
        self._lim_seen: Dict[int, int] = {}    # 本轮触发过的限幅 (nid → LIM_* 掩码)
        self._err_total: Dict[int, int] = {}   # 整场累计, 收尾时汇总
        self._lim_total: Dict[int, int] = {}   # 整场触发过的限幅, 收尾时汇总
        self._status_t = 0.0
        self._hold = [0.0] * TOTAL_JOINTS      # 未选 slot 的保位值
        self._last_cmd = [0.0] * TOTAL_JOINTS  # 最后一帧名义目标(冻结回零用)
        self._got_frame = False
        self._act_lo = [float("inf")] * TOTAL_JOINTS    # 运动循环里实际走到的极值
        self._act_hi = [float("-inf")] * TOTAL_JOINTS
        self._running = False                  # 已进入运动循环? 决定失败时停不停全场
        self._released = False                 # 动过 effort_limit → 退出时要还原
        self._ticks = self._late = 0

    def log(self, msg: str) -> None:
        print("[%s] %s" % (self.alias, msg), flush=True)

    # ── 线程入口 ────────────────────────────────────────────────────────
    def run(self) -> None:
        prof = None
        try:
            # 起跑阶段必须**串行**: SDK 的连接握手会把已连上的手的订阅回调饿死 ——
            # 实测两只手并发起跑时, 先连的那只 joint_states 回调停了十几秒
            # (SDK 报 `lagged 13586 messages`), 等不到反馈帧就自判失败。连接 / 使能 /
            # 回零都在锁里, 跑起来之后各跑各的, 互不影响。
            if not _STARTUP_LOCK.acquire(blocking=False):
                self.log("排队中, 等前面的手连接就位 ...")
                _STARTUP_LOCK.acquire()
            try:
                self._connect()
                self._wait_first_frame()
                prof = apply_overrides(self.build_profile(self), self.args)
                for note in prof.notes:
                    self.log("提示: " + note)
                if self.args.dry_run:
                    self.log("--dry-run: 只核对参数, 不使能不运动\n"
                             + describe_profile(prof, self.limits))
                    self.ok = True
                    return
                self._configure(prof)
                if prof.sequence is None:
                    self._preseed_and_enable(prof)
                    self._zero_return(prof)
            finally:
                _STARTUP_LOCK.release()
            if prof.sequence is not None:
                # 使能/回零/测量全在编排函数里。它自己逐批 scope(), 所以放在起跑锁
                # **外面** —— 一拖多时几只手并行测, 不然 5 指 × 几十秒要串成几倍。
                # `_running` 先置起来: 之后出错只停本手, 别把整场拖下水(同下面的注释)。
                self._running = True
                prof.sequence(self, prof)
            else:
                t_stop = self._motion_loop(prof)
                self._stop(prof, t_stop)
            self.ok = True
        except BaseException as exc:                      # noqa: BLE001
            self.error = "%s: %s" % (type(exc).__name__, exc)
            self.log("!! 异常: " + self.error + connect_hint(exc))
            # 一拖多的失败语义按出错时机分两种:
            #   起跑前失败(连不上/限位表不对/使能失败) → 多半是配置或操作问题,
            #     整场停下来让人当场发现, 免得跑半天才知道少一只手;
            #   跑起来后失败(掉线/网络抖) → 只停自己, 别让一只手毁掉整架老化。
            if self._running:
                self.log("!! 本手已单独停机, 其余手继续老化")
            else:
                self.log("!! 起跑前失败 —— 通知所有手停机")
                self.stop_event.set()
        finally:
            self._shutdown(prof)

    # ── 连接与反馈 ──────────────────────────────────────────────────────
    def _connect(self) -> None:
        self.log("连接 %s ..." % self.address)
        # enable_bridge 是给**多个进程共享同一只手**用的: 开了它, 本进程会起一个
        # DeviceBridge 占住设备那条唯一会话, 别的进程(watch.py)再连就挂到这个桥上,
        # 而不是去抢设备 —— 设备的并发会话数只有个位数, 第二个进程直连会被
        # close reason 3 (MAX_SESSIONS) 拒掉, 见 connect_hint()。
        #
        # 默认**关**: 关掉可以少一个 bridge 看门狗误判断链的失效面, 而长跑老化最怕的
        # 就是没验过的组件在第 N 小时掉链子。要在老化期间用 watch.py 看数据才加
        # --bridge, 并且先在台架上长跑验一轮。
        bridge = bool(getattr(self.args, "bridge", False))
        self.hand = SdkManager.instance().connect(
            address=self.address, device_name=self.alias,
            options=ConnectOptions(timeout_ms=3000, retry_count=3, enable_bridge=bridge))
        self.serial_number = self.hand.serial_number
        try:
            self.handedness = self.hand.handedness().get()
        except Exception:                                 # noqa: BLE001
            pass
        online = self.hand.online_joints_count().get()
        self.log("已连接 SN=%s 手型=%s 在线关节=%d/20"
                 % (self.serial_number, self.handedness, online))
        if online < TOTAL_JOINTS:
            self.log("注意: %d 个关节离线, 不会被使能, 命令帧里填保位值。"
                     % (TOTAL_JOINTS - online))
        self._select_limits()
        # 反馈用**回调订阅**: joint_states 原生 ~970Hz 而命令循环只有 200Hz,
        # 用 subscribe()+recv() 在主循环里收会每拍溢缓冲(SDK 刷 `lagged` 警告)。
        # 回调由 SDK 后台线程按原生速率收, 我们只留最新值。
        self._sub = self.hand.joint_states().subscribe_with_callback(self._on_state)
        self._pub = self.hand.joint_command().publish()
        # 诊断流常开: 老化一跑几小时, 中间没有任何输出就没法判断是否正常。
        # 母线电压 / 电流 / 故障码也是出问题时最有用的三个信号。
        self._diag_sub = self.hand.joint_diagnostics().subscribe_with_callback(
            self._on_diag)

    def _track_reset(self) -> None:
        self._act_lo = [float("inf")] * TOTAL_JOINTS
        self._act_hi = [float("-inf")] * TOTAL_JOINTS
        # 温升基线: 绝对温度受"这只手之前跑了多久"影响太大, 判因要看本轮涨了多少,
        # 以及**哪些轴涨得最多** —— 后者比绝对值稳健得多, 不怕起始温度不一致。
        self._temp_base = dict(self._temp_by_nid)
        self._temp_by_nid = {}

    def _log_tracking(self, prof: Profile) -> None:
        """对比**命令行程**和**实际走到的行程** —— 命令发出去不等于关节跟得上。

        小振幅 + 低 kp 的轴(典型: 四指外展摆动)可能被静摩擦吃掉大半, 光看日志里的
        「±5°」会以为在动, 肉眼却看不出来。这里用反馈实测把差距摆出来。
        """
        cmd_lo, cmd_hi = sample_motion_range(prof)
        bad = []
        ok = 0
        for k in prof.idxs():
            span_cmd = cmd_hi[k] - cmd_lo[k]
            span_act = self._act_hi[k] - self._act_lo[k]
            if span_cmd < 0.5 * D2R:                  # 命令本来就不动的轴, 不评价
                continue
            ratio = span_act / span_cmd if span_cmd > 0 else 0.0
            if ratio < 0.8:
                bad.append((joint_label(k), span_cmd * R2D, span_act * R2D, ratio * 100.0))
            else:
                ok += 1
        if not bad:
            self.log("跟踪检查: %d 个运动轴实际行程都 ≥ 命令的 80%%" % ok)
            return
        self.log("跟踪检查: %d 个轴实际行程 < 命令的 80%% (其余 %d 个正常):" % (len(bad), ok))
        for label, c, a, r in bad:
            self.log("    %-14s 命令 %5.1f° → 实际 %5.1f°  (%.0f%%)" % (label, c, a, r))

    def _export_limits(self, prof: Profile, path_tmpl: str) -> None:
        """把实测到的**真限位**导出成限位表, 供下次 --limits 用。

        判据是这里的要害 —— 只有**被固件夹住**的那一侧才采信实测值:
        某个轴跟得好好的时候, 它的实际极值就等于命令极值 = 设计限位 - 5° 余量,
        照抄进表里就等于每跑一轮把限位往里缩 5°, 越缩越窄。所以逐**侧**判断:
        该侧实际没走到命令角(差 > CLAMP_TOL) 且该轴报过位置限位 → 才是被夹住,
        用实测值; 否则保留原设计值。
        """
        CLAMP_TOL = 2.0 * D2R
        cmd_lo, cmd_hi = sample_motion_range(prof)
        ranges, n_meas, n_keep = {}, 0, 0
        for k in JOINTS:
            design = self.limits.get(k)
            if design is None:
                continue
            lo, hi = design
            nid = flat_to_nid(k)
            clamped = bool(self._lim_total.get(nid, 0) & LIM_POS)
            side = []
            if clamped and prof.enabled[k]:
                if self._act_lo[k] - cmd_lo[k] > CLAMP_TOL:      # 下端被顶住
                    lo = self._act_lo[k]
                    side.append("min")
                if cmd_hi[k] - self._act_hi[k] > CLAMP_TOL:      # 上端被顶住
                    hi = self._act_hi[k]
                    side.append("max")
            if side:
                n_meas += 1
                src = "实测 %s (本轮)" % "+".join(side)
            elif self.limits.is_measured(k):
                # 本轮没被夹住, 但基准表里这个值本来就是实测出来的 —— 不能标成"设计值",
                # 否则迭代几轮之后就分不清哪些是真限位、哪些是图纸值了。
                n_keep += 1
                src = "实测 (沿用 %s)" % os.path.basename(self.limits.path)
            else:
                src = "设计值 (本轮未被限位夹住)"
            ranges[str(nid)] = {
                "min_rad": round(lo, 6), "max_rad": round(hi, 6),
                "name": joint_label(k), "source": src,
            }
        if n_meas + n_keep == 0:
            # 一个轴都没量到 → 导出的表和设计表逐字相同, 却会被下次加载标成"本手实测",
            # 反而误导。不如不写, 并说清楚为什么。
            self.log("未导出限位表: 本轮没有任何关节被限位夹住, 量不到真限位。"
                     "要标定请用满行程跑 (--amp-scale 1, 别缩行程)。")
            return
        stem, ext = os.path.splitext(path_tmpl)
        out = "%s_%s%s" % (stem, self.serial_number, ext or ".json")
        doc = {"_meta": {
            "version": 2, "policy": "measured_from_clamp",
            "serial_number": self.serial_number, "handedness": self.handedness,
            "source": "老化跑动中被固件夹住的实测极值; 未夹住的轴沿用 %s"
                      % os.path.basename(self.limits.path),
            "comment": "逐关节 source 字段的三种取值: 「实测 …(本轮)」= 这轮被固件夹住量到的; "
                       "「实测 (沿用 …)」= 之前某轮量到的, 本轮没再被夹住; "
                       "「设计值 …」= 从没量到过, 仍是图纸值。"
                       "要更准就用满行程(--amp-scale 1)跑一轮再导, 可反复迭代收敛。",
        }, "ranges": ranges}
        with open(out, "w", encoding="utf-8") as fp:
            json.dump(doc, fp, ensure_ascii=False, indent=2)
        self.log("已导出限位表 %s (本轮新测 %d 轴, 沿用既有实测 %d 轴, 仍是设计值 %d 轴)"
                 % (out, n_meas, n_keep, len(ranges) - n_meas - n_keep))

    def _select_limits(self) -> None:
        """按 SN 给**这只手**挑限位表。一拖多时每只手各挑各的。

        优先级: --limits 显式指定 > limits_<SN>.json(本手实测) > joint_limits.json(设计值)
        """
        if self.args.limits:
            # 显式指定就照办(可能是故意共用一份), 但对不上号要说一声。
            if self.limits.serial_number and self.limits.serial_number != self.serial_number:
                self.log("⚠ 限位表 %s 是 %s 的, 当前手是 %s —— 行程会不准, 确认是有意共用。"
                         % (os.path.basename(self.limits.path),
                            self.limits.serial_number, self.serial_number))
            return
        p = JointLimits.find_for_sn(self.serial_number)
        if p:
            try:
                self.limits = JointLimits(p)
                n = sum(1 for k in JOINTS if self.limits.is_measured(k))
                self.log("限位表: %s (本手专属, %d/20 轴为实测真限位, 其余仍是设计值)"
                         % (os.path.basename(p), n))
                return
            except (OSError, ValueError, KeyError) as exc:      # noqa: BLE001
                self.log("本手限位表 %s 读取失败(%s), 退回设计值" % (p, exc))
        self.log("限位表: %s —— **设计值, 不是这只手的实测限位**。部分轴可能顶限位; "
                 "跑一轮 `--export-limits` 就能标定出 %s。"
                 % (os.path.basename(self.limits.path),
                    JointLimits.PER_HAND_FMT % self.serial_number))

    def _on_state(self, frame) -> None:
        """joint_states 回调(跑在 SDK 后台线程)。只做赋值, 别在这里干重活。

        帧里**只有在线关节**(变长, 按 nid 查), 所以出现过 = 在线。
        """
        for j in frame.joints:
            k = nid_to_flat(j.nid)
            if k >= 0:
                self.actual[k] = j.position
                self.online[k] = True
                if j.position < self._act_lo[k]:
                    self._act_lo[k] = j.position
                if j.position > self._act_hi[k]:
                    self._act_hi[k] = j.position
        self._got_frame = True

    def _on_diag(self, frame) -> None:
        """joint_diagnostics 回调(SDK 后台线程)。留最新一帧汇总 + 累积故障/限幅。

        故障码是固件原样上报的 u16(含告警位), 公开 SDK 没给码表, 所以这里原样打十六进制;
        真正说明「为什么」的是状态字里的三个限幅标志和母线电压/温度 —— 它们指得出是
        堵转限流、撞到软限位, 还是热了。
        """
        worst_v, max_i, max_t, errs, lims = 99.0, 0.0, 0.0, {}, {}
        hot_nid = -1
        for j in frame.joints:
            worst_v = min(worst_v, j.vbus_v_fb)
            max_i = max(max_i, abs(j.current))
            t_j = j.mcu_temp_c_fb
            if t_j > max_t:
                max_t, hot_nid = t_j, j.nid
            # 逐关节温度峰值: 只报全手最高值的话, 分不清是「某个轴在堵转发热」还是
            # 「整手都在正常升温」—— 而这两件事的处理完全不同。
            if t_j > self._temp_by_nid.get(j.nid, -99.0):
                self._temp_by_nid[j.nid] = t_j
            sw = j.status_word
            mask = ((LIM_POS if sw.position_limit_active else 0)
                    | (LIM_VEL if sw.velocity_limit_active else 0)
                    | (LIM_CUR if sw.current_limit_active else 0))
            if mask:
                lims[j.nid] = mask
                self._lim_seen[j.nid] = self._lim_seen.get(j.nid, 0) | mask
                self._lim_total[j.nid] = self._lim_total.get(j.nid, 0) | mask
            code = j.error_code_current
            if code:
                # 有码时才取 ext_state_name —— 它是属性调用, 别每帧 20 个关节都算。
                errs[j.nid] = (code, sw.ext_state_name)
                self._err_seen[j.nid] = self._err_seen.get(j.nid, 0) | code
                self._err_total[j.nid] = self._err_total.get(j.nid, 0) | code
        # 极值在这里累(~1kHz), 不放到 60s 一次的状态行里 —— 否则短跑或尖峰会漏掉。
        # 电流尤其如此: 只报最新一帧的最大值等于随机抓拍(正弦换向时恰好接近 0),
        # 要的是**区间峰值** —— _i_win 每打一行状态清零, _i_peak 留给整场汇总。
        self._vbus_min = min(self._vbus_min, worst_v)
        self._temp_max = max(self._temp_max, max_t)
        self._i_win = max(self._i_win, max_i)
        self._i_peak = max(self._i_peak, max_i)
        self._diag = (worst_v, max_i, max_t, hot_nid, errs, lims)

    @staticmethod
    def _code_txt(c: int) -> str:
        d = describe_code(c)
        return "0x%04X(%s)" % (c, d) if d else "0x%04X" % c

    def _fmt_codes(self, seen: Dict[int, int]) -> str:
        return ", ".join("%s=%s" % (joint_label(nid_to_flat(n)), self._code_txt(c))
                         for n, c in sorted(seen.items()) if nid_to_flat(n) >= 0)

    def _log_status(self, elapsed_s: float) -> None:
        """老化中每 STATUS_INTERVAL_S 打一行进度 + 健康度。

        母线电压塌陷(掉到 10V 以下)= 多关节堵转或相撞把电流拉满的典型征兆,
        比"跑着跑着掉线"这种表象有用得多。
        """
        if not self._diag:
            return
        worst_v, _max_i, max_t, hot_nid, errs, lims = self._diag
        i_win, self._i_win = self._i_win, 0.0          # 本窗口峰值, 取完清零
        hot = joint_label(nid_to_flat(hot_nid)) if nid_to_flat(hot_nid) >= 0 else "?"
        txt = ("已跑 %.0f 分钟 | 母线 %.1fV(最低 %.1fV) 电流峰 %.2fA 温度 %.0f℃@%s(最高 %.0f℃)"
               % (elapsed_s / 60.0, worst_v, self._vbus_min, i_win, max_t, hot, self._temp_max))

        # 限幅标志: 现在正踩着的用关节名列出, 本轮踩过又松开的只报个数(免得刷屏)。
        lim_seen, self._lim_seen = self._lim_seen, {}
        for bit, name in ((LIM_CUR, "限流"), (LIM_POS, "限位"), (LIM_VEL, "限速")):
            now = [joint_label(nid_to_flat(n)) for n, m in sorted(lims.items())
                   if m & bit and nid_to_flat(n) >= 0]
            past = sum(1 for m in lim_seen.values() if m & bit) - len(now)
            if now:
                txt += " | %s: %s%s" % (name, ", ".join(now),
                                        "(另有 %d 个曾触发)" % past if past > 0 else "")
            elif past > 0:
                txt += " | %s: 本分钟内 %d 个关节曾触发" % (name, past)

        # 错误码: 现存的带状态名, 闪现又自清的单独列 —— 后者是最容易漏掉的早期征兆。
        # 注意这个码**含告警位**(SDK: u16, includes warnings), 非零不等于故障,
        # 所以这里只叫「错误码」不叫「故障」, 判不判停机交给看的人。
        err_seen, self._err_seen = self._err_seen, {}
        if errs:
            txt += " | 错误码: " + ", ".join(
                "%s=%s[%s]" % (joint_label(nid_to_flat(n)), self._code_txt(c), s)
                for n, (c, s) in sorted(errs.items()) if nid_to_flat(n) >= 0)
        cleared = {n: c for n, c in err_seen.items() if n not in errs}
        if cleared:
            txt += " | 错误码(已自清): " + self._fmt_codes(cleared)
        self.log(txt)

    def online_idxs(self) -> List[int]:
        """当前在线的关节 flat 下标。档案构建时用它决定跑哪些关节。"""
        return [k for k in JOINTS if self.online[k]]

    def _wait_first_frame(self, timeout_s: float = 3.0) -> None:
        """没有实际角就不能安全预置 —— 等不到就报错, 不硬着头皮使能。"""
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            if self._got_frame:
                return
            time.sleep(0.005)
        raise RuntimeError("等不到 joint_states 反馈帧, 拒绝使能")

    # ── 配置 / 预置 / 使能 ──────────────────────────────────────────────
    def _configure(self, prof: Profile) -> None:
        self.log("清故障 + 设参数: kp=%.2f kd=%.3f iq=%.2fA" % (prof.kp, prof.kd, prof.iq))
        # 先整手 disable 拿个干净起点: 上一次会话可能把某些关节留在使能态, 那些关节
        # 不在本次 enable 掩码里, 却仍会吃到广播帧 —— 先关掉最省心。此刻电机本就该
        # 是断电的, 这一步不产生动作。(对齐上位机 doStartSingleJointAging 的 disableAll)
        self._try(self.hand.disable)
        try:
            self.hand.clear_fault()
        except Exception as exc:                          # noqa: BLE001
            self.log("clear_fault 跳过 (%s)" % exc)
        # SET 要重试: joint_states(~1kHz) + joint_diagnostics 两条回调流已经订上了,
        # 这时候发 SET 查询偶发 `SET error: set failed: InternalError` —— 设备侧没故障
        # (单独连、不订阅时同样的 SET 百发百中), 是并发下的瞬时失败。
        # 起跑前崩一次就白等一场老化, 所以退避重试而不是直接抛。
        self._set_retry("mit_params", lambda: self._set_mit(prof.kp, prof.kd))
        self._set_retry("effort_limit",
                        lambda: self.hand.effort_limit().set(float(prof.iq)))

    def _set_retry(self, name: str, fn, tries: int = 4, delay: float = 0.4) -> None:
        """SET 类调用的退避重试。最后一次仍失败才抛 —— 参数没设上就不能使能。"""
        for i in range(1, tries + 1):
            try:
                fn()
                if i > 1:
                    self.log("%s 第 %d 次重试成功" % (name, i))
                return
            except Exception as exc:                          # noqa: BLE001
                if i == tries:
                    raise
                self.log("%s 设置失败(%s), %.1fs 后重试 %d/%d"
                         % (name, exc, delay, i + 1, tries))
                time.sleep(delay)

    def _set_mit(self, kp: float, kd: float) -> None:
        """公开 SDK 的 mit_params 是整手资源, 一个 (kp,kd) 即全 20 关节。"""
        self.hand.mit_params().set((float(kp), float(kd)))

    def _preseed_and_enable(self, prof: Profile) -> None:
        """安全纪律 #1: 先用 target=actual 播满 100ms 再通电, 通电后再保位 50ms
        压住上电瞬态, 之后才让回零斜坡接手。"""
        self._hold = list(self.actual)
        self.log("预置保位 %d 帧 (target=actual, 电机未通电)" % PRESEED_FRAMES)
        self._hold_frames(PRESEED_FRAMES)

        idxs = prof.idxs()
        self.log("使能 %d 个关节: %s" % (len(idxs), ", ".join(joint_label(k) for k in idxs)))
        self.hand.enable(joints=prof.mask())
        self._hold_frames(POST_ENABLE_FRAMES)

    def _hold_frames(self, n: int) -> None:
        for _ in range(n):
            self._send(self._hold, _ZEROS, _ZEROS)
            time.sleep(FRAME_INTERVAL_S)

    # ── 逐批使能 (给 SequenceFn 用) ─────────────────────────────────────
    def scope(self, prof: Profile, enabled: Sequence[bool], label: str = "") -> None:
        """把使能范围切到一组新的 slot: 断开旧的 → 预置保位 → 使能新的。

        **为什么要分批而不是整手一次使能**(位控QC 逐指跑就是为了这个, 与上位机
        `PosPerfQcWorker` 同一个理由):
          - 20 轴齐动的母线电流会把扭矩顶到限流, 限流下测出来的跟踪延迟和静差全不可信;
          - 一次 enable 20 个关节偶发会话卡死。

        每次切都重走安全纪律 #1 和 #3: 先用 target=actual 播满预置帧再通电, 未选
        slot 填**切换瞬间**的实际角当保位值(不是 0)。
        """
        if any(prof.enabled):
            self._try(lambda: self.hand.disable(joints=prof.mask()))
        prof.enabled = list(enabled)
        self._hold = list(self.actual)
        self._hold_frames(PRESEED_FRAMES)
        idxs = prof.idxs()
        self.log("使能%s: %s" % (label or " %d 个关节" % len(idxs),
                                 ", ".join(joint_label(k) for k in idxs)))
        self.hand.enable(joints=prof.mask())
        self._hold_frames(POST_ENABLE_FRAMES)

    def unscope(self, prof: Profile) -> None:
        """断开当前这一批。断完 `enabled` 全 False —— 之后 `_emit()` 对 20 个 slot
        一律发保位值, 即使编排函数还在下发也不会动手。"""
        if any(prof.enabled):
            self._try(lambda: self.hand.disable(joints=prof.mask()))
        prof.enabled = [False] * TOTAL_JOINTS

    # ── 下发 ────────────────────────────────────────────────────────────
    def _send(self, pos, vel, eff) -> None:
        self._pub.send([JointCommand(float(pos[k]), float(vel[k]), float(eff[k]))
                        for k in JOINTS])

    def _emit(self, prof: Profile, pos: Sequence[float], vel: Sequence[float],
              eff: Optional[Sequence[float]] = None) -> None:
        """未选 slot 换成保位值(安全纪律 #3) + 速度前馈, 然后下发。

        `eff` 非 None → 力矩通道**由轨迹自己给**, 不再套 `KD_FF * vel` 的速度前馈。
        位控QC 的软件积分走这条(固件没有 ki, 静差只能靠上位机侧积分补), 而且它测
        跟踪延迟那一趟必须是**纯位置通道**: 带上速度前馈等于给了固件超前量, 测出来
        的延迟会偏小。未选 slot 的力矩一律清零, 免得给保位中的关节额外加力。
        """
        self._last_cmd = list(pos)
        p, v = list(pos), list(vel)
        e = list(eff) if eff is not None else None
        for k in JOINTS:
            if not prof.enabled[k]:
                p[k], v[k] = self._hold[k], 0.0
                if e is not None:
                    e[k] = 0.0
        if e is not None:
            self._send(p, v, e)
        elif prof.feedforward:
            self._send(p, v, [KD_FF * v[k] for k in JOINTS])
        else:
            self._send(p, _ZEROS, _ZEROS)   # 位置通道(与上位机滑条一致)

    # ── 200Hz 节拍原语 —— 所有运动阶段都走它 ────────────────────────────
    def _run(self, prof: Profile, traj, duration_s: Optional[float] = None,
             label: str = "", until=None) -> float:
        """以 200Hz 跑一段, 每拍调 `traj(t)` 取 (pos, vel) 下发。返回实际时长。

        duration_s=None → 一直跑到 stop_event(运动主循环);
        until(t)=True   → 提前结束(贴零「到位即松」)。
        """
        if label:
            self.log(label if duration_s is None else "%s (%.1fs)" % (label, duration_s))
        t0 = next_t = time.perf_counter()
        while True:
            t = time.perf_counter() - t0
            self._emit(prof, *traj(t))
            self._ticks += 1
            if t - self._status_t >= STATUS_INTERVAL_S:
                self._status_t = t
                self._log_status(t)
            done = self.stop_event.is_set() if duration_s is None else t >= duration_s
            if done or (until is not None and until(t)):
                break
            next_t += TICK_S
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif next_t < time.perf_counter() - LATE_RESYNC_S:
                next_t = time.perf_counter()   # 被调度拖太远, 重新对齐
                self._late += 1
        return time.perf_counter() - t0

    # ── 阶段原语 (给 SequenceFn 用) ─────────────────────────────────────
    def run_phase(self, prof: Profile, traj, secs: float, label: str = "") -> bool:
        """以 200Hz 跑一段**固定时长**的轨迹, 期间随时响应停机请求。

        `traj(t)` 返回 (pos, vel) 或 (pos, vel, eff) —— 见 `MotionFn`。t 从本段开头
        起算(每段都从 0 开始), 所以跨段累积的状态要自己存在闭包里。

        返回 True = 跑完; False = 中途收到停机请求(Ctrl+C / --duration 到时),
        编排函数**应当立刻收尾**, 别再往下测。
        """
        self._run(prof, traj, secs, label, until=lambda _t: self.stop_event.is_set())
        return not self.stop_event.is_set()

    def ramp_phase(self, prof: Profile, dst: Sequence[float], secs: float,
                   label: str = "") -> bool:
        """从**当前实际角**余弦斜坡到 dst(两端速度为 0)。返回值同 `run_phase()`。"""
        return self.run_phase(prof, self._ramp(list(self.actual), list(dst), secs),
                              secs, label)

    def limit_flags(self, nid: int) -> int:
        """该关节**最新一帧**的限幅标志 (LIM_POS/LIM_VEL/LIM_CUR 按位或)。

        给 SequenceFn 判「这次测量还算不算数」用: 命令角超出这只手的真限位时固件会
        把它夹住, 夹住期间量到的静差是「命令角 - 限位角」, 不是伺服误差 —— 拿它
        判合格就是误判。注意取的是**最新一帧**, 要覆盖一整段就每拍 OR 一次。
        """
        d = self._diag
        return 0 if not d else int(d[5].get(nid, 0))

    # ── 轨迹片段 ────────────────────────────────────────────────────────
    @staticmethod
    def _ramp(src: Sequence[float], dst: Sequence[float], dur: float):
        """余弦 S 曲线 src→dst, 两端速度为 0 → 起步和到位都温和, 无速度阶跃。"""
        def traj(t):
            a = min(1.0, t / dur)
            s = 0.5 * (1.0 - math.cos(a * PI))
            w = (0.5 * PI / dur) * math.sin(a * PI)     # ds/dt 的形状
            return ([src[k] * (1.0 - s) + dst[k] * s for k in JOINTS],
                    [(dst[k] - src[k]) * w for k in JOINTS])
        return traj

    @staticmethod
    def _still(target: Sequence[float]):
        """守住一个姿态不动。"""
        return lambda t: (target, _ZEROS)

    # ── 阶段 ────────────────────────────────────────────────────────────
    def _zero_return(self, prof: Profile) -> None:
        self._run(prof, self._ramp(list(self.actual), prof.center, ZERO_DURATION_S),
                  ZERO_DURATION_S, "回零/就位斜坡")

    def _motion_loop(self, prof: Profile) -> float:
        """无限循环下发轨迹。返回停止时刻的 t —— 收摆段要接着这个相位往下走。"""
        self.log("开始老化循环 —— Ctrl+C 停止 (会自动收摆回零后断使能)")
        self._running = True
        self._track_reset()          # 只统计老化循环里的实际行程, 不含回零斜坡
        t0_ticks, t0_late = self._ticks, self._late

        def traj(t):
            env = 1.0 if prof.ramp_s <= 0.0 else min(1.0, t / prof.ramp_s)
            return prof.motion(t, env)

        elapsed = self._run(prof, traj)
        self.log("老化循环结束, 累计 %.1f 分钟 (%d 拍, 掉拍 %d 次)"
                 % (elapsed / 60.0, self._ticks - t0_ticks, self._late - t0_late))
        self._log_tracking(prof)
        if getattr(self.args, "export_limits", None):
            self._try(lambda: self._export_limits(prof, self.args.export_limits))
        # 整场台账: 长跑的日志没人会翻, 收尾时汇总一行才看得见。限幅也必须进汇总 ——
        # 状态行 60s 才一行, 短于一个间隔的试跑(小幅确认剐蹭那种)一行都打不出来,
        # 只看汇总会误以为"没触发限流"。
        lim_txt = []
        for bit, name in ((LIM_CUR, "限流"), (LIM_POS, "限位"), (LIM_VEL, "限速")):
            hit = [joint_label(nid_to_flat(n)) for n, m in sorted(self._lim_total.items())
                   if m & bit and nid_to_flat(n) >= 0]
            if hit:
                lim_txt.append("%s(%s)" % (name, ", ".join(hit)))
        hot3 = sorted(self._temp_by_nid.items(), key=lambda kv: -kv[1])[:3]
        hot_txt = ", ".join("%s %.0f℃" % (joint_label(nid_to_flat(n)), t)
                            for n, t in hot3 if nid_to_flat(n) >= 0)
        rise = {n: t - self._temp_base[n] for n, t in self._temp_by_nid.items()
                if n in self._temp_base and nid_to_flat(n) >= 0}
        if rise:
            top = sorted(rise.items(), key=lambda kv: -kv[1])[:3]
            self.log("本轮温升最大三轴: " + ", ".join(
                "%s +%.1f℃" % (joint_label(nid_to_flat(n)), d) for n, d in top))
        self.log("整场汇总: 母线最低 %.1fV, 电流峰 %.2fA, 最热三轴 %s, %s, %s"
                 % (self._vbus_min, self._i_peak, hot_txt,
                    ("全程触发过 " + " ".join(lim_txt)) if lim_txt else "全程无限幅",
                    ("出现过错误码 —— " + self._fmt_codes(self._err_total))
                    if self._err_total else "全程无错误码"))
        return elapsed

    def _stop(self, prof: Profile, t_stop: float) -> None:
        if prof.stop_style == "freeze":
            self._stop_freeze(prof)
        else:
            self._stop_sine(prof, t_stop)

    def _stop_sine(self, prof: Profile, t_stop: float) -> None:
        """正弦类(空载/带载)四段式停机, 对齐上位机。

        ① 沿**原正弦轨迹**用余弦包络把摆幅收到 0 —— 包络起始斜率为 0, 停止瞬间
           速度连续, 手指沿自己的轨迹收摆而不是被拽停, 不会冲出去;
        ② 在正弦中心停稳, 吸收残余加速度(不停稳直接回零就是「震一下」的来源);
        ③ 从静止柔顺回 0(全伸展); ④ 高 kp 贴零收残差。
        """
        self._run(prof,
                  lambda tau: prof.motion(
                      t_stop + tau, 0.5 * (1.0 + math.cos(PI * tau / ABORT_RAMPDOWN_S))),
                  ABORT_RAMPDOWN_S, "[停止] 沿原轨迹收摆")
        self._set_mit(ZERO_KP, ZERO_KD)
        self._run(prof, self._still(prof.center), ABORT_HOLD_S, "[停止] 停稳保持")
        self._run(prof, self._ramp(prof.center, _ZEROS, ABORT_RETURN_S),
                  ABORT_RETURN_S, "[停止] 柔顺回零")
        self._settle(prof)

    def _stop_freeze(self, prof: Profile) -> None:
        """梯形类(单指负载)停机: 冻结当前目标 smoothstep 回 0, 再缓释放扭矩。"""
        src, dur = list(self._last_cmd), prof.freeze_ramp_s
        self._run(prof,
                  lambda t: ([src[k] * (1.0 - smoothstep(t / dur)) for k in JOINTS], _ZEROS),
                  dur, "[停止] 冻结目标, 回零")
        if prof.release_s > 0.0:
            self._soft_release(prof)
        else:
            self._run(prof, self._still(_ZEROS), 0.3, "[停止] 零位保持")

    def _soft_release(self, prof: Profile) -> None:
        """扭矩上限缓降后再断使能。

        disable 是二值的(直接断 PWM), 挂着负重块的手指会自由落体砸下去。位置守在
        0 不动, 把 effort_limit 分档降到地板值 → 重力在一个衰减的扭矩天花板下把
        手指慢慢带下去。effort_limit 的 SET 只写固件 RAM 镜像 + 热同步电流环,
        **不写 flash**, 反复刷不磨寿命。只写本次使能的关节, 不碰离线关节。
        """
        idxs, step_s = prof.idxs(), prof.release_s / prof.release_steps
        self.log("[停止] 扭矩上限缓释放 %.2fA → %.2fA, %d 档 / %.1fs"
                 % (prof.iq, prof.release_floor_a, prof.release_steps, prof.release_s))
        for step in range(1, prof.release_steps + 1):
            frac = 1.0 - step / float(prof.release_steps)
            amps = max(0.0, prof.release_floor_a + (prof.iq - prof.release_floor_a) * frac)
            for k in idxs:
                self.hand.joint(k).effort_limit().set(amps)
            self._released = True
            self._run(prof, self._still(_ZEROS), step_s)

    def _settle(self, prof: Profile) -> None:
        """贴零: 高 kp 顶住 0 位收残差。全部到位就提前松手 —— 高 kp 顶零点每多一拍
        就多一拍 PWM 蜂鸣, duration 只作超时兜底。"""
        self._set_mit(SETTLE_KP, SETTLE_KD)
        idxs = prof.idxs()
        done = lambda t: all(abs(self.actual[k]) < SETTLE_TOL_RAD for k in idxs)
        used = self._run(prof, self._still(_ZEROS), SETTLE_S,
                         "[停止] 贴零 (kp=%.0f)" % SETTLE_KP, until=done)
        if used < SETTLE_S:
            self.log("[停止] 残差全部 < %.1f°, 提前收手" % (SETTLE_TOL_RAD * R2D))

    # ── 收尾 ────────────────────────────────────────────────────────────
    def _shutdown(self, prof: Optional[Profile]) -> None:
        """安全纪律 #4: 任何退出路径都走到这里。每步单独 try —— 前面失败不能
        挡住后面的 disable。"""
        if self.hand is None or self.args.dry_run:
            self._close()
            return
        try:
            if prof is not None and not prof.disable_all_on_exit:
                self.hand.disable(joints=prof.mask())
            else:
                # 还没建出档案, 或档案自己要求整手断(逐批使能的流程 —— 此刻 mask 里
                # 只剩最后一批, 按它断会漏掉前面几批) → 整手断。
                self.hand.disable()
            self.log("已断使能")
        except Exception as exc:                          # noqa: BLE001
            self.log("disable 失败 (%s), 兜底整手 disable" % exc)
            self._try(self.hand.disable)
        # 缓释放降过扭矩上限就要还原, 且必须在 disable **之后** —— 否则等于断电前
        # 又把上限拉满, 白费一段缓降。
        if self._released and prof is not None:
            try:
                for k in prof.idxs():
                    self.hand.joint(k).effort_limit().set(float(prof.iq))
                self.log("扭矩上限已还原 %.2fA" % prof.iq)
            except Exception as exc:                      # noqa: BLE001
                self.log("扭矩上限还原失败 (%s)" % exc)
        self._close()

    def _close(self) -> None:
        for stream in (self._pub, self._sub, self._diag_sub):
            if stream is not None:
                self._try(stream.close)
        if self.hand is not None:
            self._try(self.hand.disconnect)
            self.log("已断开")

    @staticmethod
    def _try(fn) -> None:
        try:
            fn()
        except Exception:                                 # noqa: BLE001
            pass


# ──────────────────────────────────────────────────────────────────────────
# 多手编排 + 命令行
# ──────────────────────────────────────────────────────────────────────────

def describe_code(code: int) -> str:
    """把固件故障码解成人话, 解不出就返回空串。

    SDK 2026.8.17 起 `WujiHand2.describe_error(code)` 提供官方码表(name/severity/
    cause/resolution)。老版本没有这个静态方法 —— 用 getattr 探测, 缺了就退回只打
    十六进制, 不因为 SDK 版本不同而崩。
    """
    fn = getattr(getattr(wuji_sdk, "WujiHand2", None), "describe_error", None)
    if fn is None:
        return ""
    try:
        info = fn(code)
    except Exception:                                     # noqa: BLE001
        return ""
    if info is None:
        return ""
    # 2026.8.17 实测返回的是 **dict**(而 .pyi 文档写的是"对象,暴露 name/severity/..."),
    # 两种都兜住, 免得下个版本改回对象又炸。
    get = info.get if isinstance(info, dict) else lambda k, d="": getattr(info, k, d)
    name = str(get("name", "") or "")
    sev = str(get("severity", "") or "")
    if not name:
        return ""
    return "%s/%s" % (name, sev) if sev else name


def connect_hint(exc: BaseException) -> str:
    """把 SDK 那句不知所云的连接失败翻成操作员能照着做的一句话。

    SDK 抛的是 `Failed to get Zenoh capabilities: ... No reply from Zenoh queryable`,
    真正的原因写在它自己打的 ERROR 日志里(`close message (reason N)`), 而且默认
    log level 下操作员看到的就是一串 Rust 源码路径。reason 3 = MAX_SESSIONS
    (zenoh-protocol `transport/close.rs`), 意思是**设备的并发会话满了**。

    实测: 手被**上位机连着**的时候就会一直报这个 —— 设备的并发会话数很少, 上位机
    占着就轮不到 SDK。断电重启没用(重启完上位机会自动连回去), 得先在上位机上断开。
    """
    msg = str(exc)
    if "Zenoh" not in msg:
        return ""
    return ("\n   连不上设备。按可能性从高到低:\n"
            "     1. 日志里有 `close message (reason 3)` = MAX_SESSIONS —— 设备的并发会话\n"
            "        满了。**先看上位机是不是正连着这只手**(或别的电脑/别的窗口开着它),\n"
            "        断开它再跑。设备同时只招待很少几个客户端。\n"
            "     2. 手刚上电还没就绪 —— 等满 40 秒。\n"
            "     3. 撞 IP —— 两只同侧手同时上电, 见 README 第 6 节。\n"
            "     4. 网线/网段 —— 电脑网卡应是 192.168.1.100/24。\n"
            "   加 -v 可以看 SDK 的完整日志。")


def discover_hands() -> List[Tuple[str, str]]:
    """扫全网找 Wuji Hand 2, 返回 [(sn, address), ...]。

    别硬编端口: 老文档写 `:50001`, 新 SDK 设备实际在 `:7447`, 扫出来最准。
    """
    return [(d.sn, d.address) for d in SdkManager.instance().scan()
            if d.device_type == wuji_sdk.DeviceType.WujiHand2]


def parse_duration(text: str) -> float:
    """'90' / '90s' / '30m' / '8h' → 秒; 0 = 不限时。"""
    t = str(text).strip().lower()
    if not t:
        return 0.0
    mult = {"s": 1.0, "m": 60.0, "h": 3600.0}.get(t[-1:], 1.0)
    return float(t.rstrip("smh")) * mult


COMMON_EPILOG = """
一拖多:
  python %(prog)s --address 192.168.1.110:7447 --address 192.168.1.111:7447
  --address 可重复; 省略则自动扫描, 扫到几只手就同时跑几只。

停止:
  Ctrl+C 按**一次** —— 所有手走完整安全停机(收摆 → 停稳 → 回零 → 断使能), 约 5 秒。
  别按第二次: 第二次是强制退出, 电机会停在当前姿态。
"""


def build_argparser(description: str, examples: str = "") -> argparse.ArgumentParser:
    """各脚本共用的命令行骨架。`examples` 是本脚本专属示例, 拼在 --help 最前面。"""
    ap = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(examples.rstrip() + "\n" if examples else "") + COMMON_EPILOG)
    ap.add_argument("--address", action="append", metavar="IP:PORT",
                    help="设备地址, 可重复 = 一拖多。省略则自动扫描。端口通常是 7447")
    ap.add_argument("--limits", metavar="FILE",
                    help="强制指定关节限位表 JSON。**一般不用写** —— 默认会按每只手的 SN "
                         "自动找 limits_<SN>.json(本手实测), 找不到才退回 joint_limits.json(设计值)")
    ap.add_argument("--export-limits", nargs="?", const="limits.json", metavar="FILE",
                    help="老化跑完后把这只手**实测到的真限位**导出成 limits_<SN>.json "
                         "(不带文件名就用这个默认)。下次再跑这只手会**自动认出并加载**, "
                         "不用写 --limits。只有被固件夹住的那一侧才用实测值; "
                         "配 --amp-scale 1 满行程跑一轮采得最全, 可反复迭代收敛")
    ap.add_argument("--duration", default="0", metavar="T",
                    help="跑多久自动停机, 支持 90s / 30m / 8h; 0 = 跑到 Ctrl+C")
    ap.add_argument("--kp", type=float, help="覆盖 MIT kp")
    ap.add_argument("--kd", type=float, help="覆盖 MIT kd")
    ap.add_argument("--iq", type=float, help="覆盖扭矩上限 (A)")
    ap.add_argument("--amp-scale", type=float, default=1.0, metavar="K",
                    help="行程缩放 0..1 (默认 1 = 满行程)。**换手型/换治具/改完轨迹后"
                         "第一次跑, 先用 0.2 小幅确认方向和干涉, 再逐步放大。**"
                         "只缩摆幅, 不动中心位置")
    ap.add_argument("--dry-run", action="store_true",
                    help="只连接 + 打印本次要跑的行程, 不使能不运动")
    ap.add_argument("--bridge", action="store_true",
                    help="开多客户端桥 —— **想在老化跑着的时候用 watch.py 看实时数据就必须加"
                         "这个**。不加的话本进程独占设备那条唯一会话, watch.py 连不上"
                         "(报 MAX_SESSIONS)。代价是多一个没在长跑里验过的组件, "
                         "所以默认关; 顺序是先起老化(桥主)再起 watch.py, 先退 watch.py")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="打印 SDK 内部日志 (连不上 / 掉线时用来看细节)")
    return ap


def apply_overrides(prof: Profile, args) -> Profile:
    """把命令行覆盖项套到档案上。"""
    prof.kp = args.kp if args.kp is not None else prof.kp
    prof.kd = args.kd if args.kd is not None else prof.kd
    prof.iq = args.iq if args.iq is not None else prof.iq

    scale = args.amp_scale
    if scale != 1.0:
        # 绕 center 缩放 —— center 同时是回零终点和正弦中心, 缩放后起点仍连续,
        # 所以正弦/抓握/梯形三种波形通用。
        base, ctr = prof.motion, list(prof.center)

        def scaled(t: float, env: float):
            pos, vel = base(t, env)
            return ([ctr[k] + (pos[k] - ctr[k]) * scale for k in JOINTS],
                    [vel[k] * scale for k in JOINTS])

        prof.motion = scaled
        prof.name += " [行程 %.0f%%]" % (scale * 100.0)
        prof.notes.append("⚠ 行程已缩放到 %.0f%%, 不是满行程老化。" % (scale * 100.0))
    return prof


def sample_motion_range(prof: Profile, n: int = 400):
    """采样一个周期, 返回每个关节命令端会走到的 (最小, 最大) 弧度。

    直接采样 motion() 而不是读 center/amp —— 三种波形用同一段代码就能核对。
    """
    period = prof.period_s or 10.0
    lo, hi = [float("inf")] * TOTAL_JOINTS, [float("-inf")] * TOTAL_JOINTS
    for i in range(n + 1):
        pos, _ = prof.motion(period * i / n, 1.0)
        for k in JOINTS:
            lo[k], hi[k] = min(lo[k], pos[k]), max(hi[k], pos[k])
    return lo, hi


def describe_profile(prof: Profile, limits: JointLimits) -> str:
    """采样一个周期, 打印命令端**实际会走到**的行程和离限位还剩多少余量。

    直接采样 motion() 而不是读 center/amp —— 三种波形用同一段代码就能核对,
    也能一眼看出有没有顶到限位。
    """
    lo, hi = sample_motion_range(prof)
    period = prof.period_s or 10.0
    lines = ["档案 %s: kp=%.2f kd=%.3f iq=%.2fA, 使能 %d 关节, 周期 %.2fs"
             % (prof.name, prof.kp, prof.kd, prof.iq, len(prof.idxs()), period),
             "  %-16s %9s %9s %9s %9s %7s"
             % ("joint", "cmd_min°", "cmd_max°", "lim_min°", "lim_max°", "余量°")]
    for k in prof.idxs():
        rng = limits.get(k)
        if rng is None:
            lines.append("  %-16s %9.1f %9.1f %9s %9s %7s"
                         % (joint_label(k), lo[k] * R2D, hi[k] * R2D, "n/a", "n/a", "n/a"))
            continue
        margin = min(lo[k] - rng[0], rng[1] - hi[k]) * R2D
        lines.append("  %-16s %9.1f %9.1f %9.1f %9.1f %7.1f%s"
                     % (joint_label(k), lo[k] * R2D, hi[k] * R2D,
                        rng[0] * R2D, rng[1] * R2D, margin,
                        "" if margin >= -0.05 else "  <<< 超限!"))
    return "\n".join(lines)


def run_multi(build_profile: ProfileBuilder, args, banner: str) -> int:
    """连接所有目标手, 各起一个线程跑同一套老化, Ctrl+C 统一安全停机。"""
    wuji_sdk.set_log_level("info" if args.verbose else "warn")
    try:
        limits = JointLimits(args.limits)
    except (OSError, ValueError, KeyError) as exc:
        print("!! 关节限位表读取失败: %s" % exc)     # 给操作员看的错, 不甩 traceback
        return 2
    print(banner)
    print("限位表: %s" % limits.path)

    if args.address:
        targets = [("hand%d" % (i + 1), a) for i, a in enumerate(args.address)]
        dup = [a for i, a in enumerate(args.address) if a in args.address[:i]]
        if dup:
            print("!! --address 里有重复地址 %s —— 一只手被当成两只跑, 会有两路 200Hz "
                  "命令流打架。请每只手写一次。" % ", ".join(sorted(set(dup))))
            return 2
    else:
        print("未指定 --address, 自动扫描 ...")
        found = discover_hands()
        if not found:
            print("!! 没扫到任何 Wuji Hand 2, 检查网线/供电, 或用 --address 手动指定")
            return 2
        for sn, addr in found:
            print("  发现 %s @ %s" % (sn, addr))
        # 撞 IP 必须在这里拦死: 同侧手出厂默认 IP 相同, 两只一起上电就都答同一个地址。
        # 不拦的话下场是每只手重试 5 次连接、最后甩一串 zenoh 内部错误(close reason 3 /
        # Failed to get Zenoh capabilities), 从那串东西反推不出真正的原因。
        by_addr: Dict[str, List[str]] = {}
        for sn, addr in found:
            by_addr.setdefault(addr, []).append(sn)
        clash = {a: sns for a, sns in by_addr.items() if len(sns) > 1}
        if clash:
            print("!! 撞 IP —— 下面这些地址被多只手同时占着, 谁都连不上:")
            for addr, sns in sorted(clash.items()):
                print("     %s  ←  %s" % (addr, ", ".join(sorted(sns))))
            print("   同侧手的出厂默认 IP 是一样的(左手都是 .110, 右手都是 .111)。")
            print("   逐只改: 只给其中一只上电, 跑 `python set_hand_ip.py --ip 192.168.1.112`,")
            print("   改完再把所有手上电重跑本脚本。详见 README 第 6 节。")
            return 2
        targets = found

    stop_event = threading.Event()
    hits = [0]

    def on_sigint(_sig, _frm):
        hits[0] += 1
        if hits[0] == 1:
            print("\n>> 收到 Ctrl+C, 所有手开始安全停机(收摆 → 回零 → 断使能), 请勿再按")
            stop_event.set()
        else:
            print("\n>> 再次 Ctrl+C —— 强制退出, 电机可能停在当前姿态!")
            os._exit(130)

    signal.signal(signal.SIGINT, on_sigint)

    runners = [HandRunner(addr, alias, build_profile, stop_event, limits, args)
               for alias, addr in targets]
    for r in runners:
        r.start()

    duration = parse_duration(args.duration)
    if duration > 0:
        print(">> 计划运行 %.1f 分钟后自动停机" % (duration / 60.0))
    deadline = (time.perf_counter() + duration) if duration > 0 else None

    # 主线程只做「短睡 + 轮询」, 不用 stop_event.wait(duration):
    #   1. Windows 上 Event.wait()/lock.acquire() **不响应 Ctrl+C**, 一睡一小时就按不停;
    #      time.sleep(0.2) 是可中断的, SIGINT 处理函数能及时跑到。
    #   2. 所有手都挂了就该立刻收尾, 而不是干等满 --duration。
    try:
        while any(r.is_alive() for r in runners):
            if deadline is not None and not stop_event.is_set() \
                    and time.perf_counter() >= deadline:
                print("\n>> 到时, 开始安全停机")
                stop_event.set()
            time.sleep(0.2)
    except KeyboardInterrupt:
        # 兜底: 万一 SIGINT 没被上面的 handler 截住(平台差异), 这里也要停下来。
        print("\n>> 收到 Ctrl+C, 所有手开始安全停机")
        stop_event.set()

    if not stop_event.is_set() and any(not r.ok for r in runners):
        print(">> 所有手都已停止, 提前收尾")
    for r in runners:
        r.join(timeout=30.0)

    failed = [r for r in runners if not r.ok]
    print("\n===== 结束: %d/%d 只手正常收尾 ====="
          % (len(runners) - len(failed), len(runners)))
    for r in failed:
        print("  %s (%s) 失败: %s" % (r.alias, r.address, r.error))
    return 1 if failed else 0
