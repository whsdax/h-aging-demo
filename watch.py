#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读监视 -- 老化跑着的时候, 另开一个终端看实时数据。想看就开, 不看就关。

**不使能、不下发、不动手, 也不改任何配置。** 只订阅反馈流并打印:
关节角 / 温度 / 电流 / 母线电压 / 限幅标志 / 错误码。

=== 前置条件: 老化脚本要带 --bridge 起, 否则本脚本连不上 ===

设备的并发会话数只有个位数, **第二个进程直连会被 close reason 3 (MAX_SESSIONS)
拒掉** -- 谁先连上谁占着。绕开的办法是 SDK 的 DeviceBridge: 第一个进程开
`ConnectOptions(enable_bridge=True)` 时会起一个桥占住设备那条会话, 并把设备的资源
在本机 Zenoh 上重新导出; 后面的进程再连就挂到这个桥上(SDK 日志: `New bridge already
exists, reconnecting via Zenoh`), 设备侧始终只有一条链。

本脚本自己一律 `enable_bridge=True`。**老化侧默认是关的**, 要加 `--bridge`:

    python no_load_aging.py --bridge          # 桥主, 先起
    python watch.py                           # 挂桥, 后起

老化那边不带 --bridge 的话它独占设备那条会话, 本脚本无论如何都连不上(会报
MAX_SESSIONS)。默认关是有意的 -- 见 aging_common.py `HandRunner._connect` 的注释。

顺序要求 -- 先起老化, 后起监视; 退出时先退监视:

    OK  老化脚本(桥主) -> watch.py(挂桥) -> 关 watch.py -> 老化继续跑
    NG  watch.py 先起  -> 它成了桥主 -> 老化挂在它身上 -> 关 watch.py 老化跟着断链

反过来那条路径没有在真机上验证过, 别赌。没在跑老化的时候单独用本脚本是安全的
(它就是唯一的客户端, 不需要谁先开桥)。

=== 刷新频率 (--interval) 由使用方自己定 ===

`--interval` **只决定屏幕多久刷一次, 不改设备的推送速率**。设备流是满速
(joint_states / joint_diagnostics 各约 970Hz), 本脚本用回调在后台线程收、只留最新值,
主循环按 interval 打印 -- 所以:

  - interval 调小不会给设备加负担(设备该发多少还是发多少), 只是刷屏更密;
  - interval 调大**也不会漏掉瞬时事件**: 电流/温度峰值、限幅标志、错误码都在
    「上一屏到这一屏」这个窗口内**累积**后再打印(这正是 0.2s 一闪的堵转限流最容易被
    "只看最新一帧"漏掉的地方)。关节角是连续量, 显示的是瞬时值, 不做峰值保持。

一屏 20 行 x 很多只手会刷不过来, 手多时用 `--compact` (每只手一行)。

用法:
    python watch.py                                   # 自动扫描, 1s 刷一次
    python watch.py --interval 0.5                    # 0.5s 刷一次
    python watch.py --address 192.168.1.110:7447 --address 192.168.1.111:7447
    python watch.py --compact --interval 2            # 多只手, 每手一行
    python watch.py --once                            # 打一屏就退出(给自动化取快照用)
    python watch.py --duration 30m                    # 看 30 分钟自动退出
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

import wuji_sdk
from wuji_sdk import ConnectOptions, SdkManager

# 用脚本自身所在目录, 不用 cwd -- 自动化设备大概率从别的目录调起本脚本。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from aging_common import (  # noqa: E402
    LIM_CUR, LIM_POS, LIM_VEL, R2D, TOTAL_JOINTS,
    connect_hint, describe_code, discover_hands, joint_label, nid_to_flat,
    parse_duration,
)

MIN_INTERVAL_S = 0.05       # 再密下去就只是在刷屏, 且打印本身开始吃 CPU
FIRST_FRAME_TIMEOUT_S = 5.0


class HandWatch:
    """一只手的只读监视: 两条订阅 + 窗口累积器。"""

    def __init__(self, alias: str, address: str):
        self.alias, self.address = alias, address
        self.hand = None
        self.serial_number = "?"
        self.handedness = "unknown"
        self.online_count = 0
        self._state_sub = self._diag_sub = None

        self.pos: List[float] = [0.0] * TOTAL_JOINTS    # 关节侧实际角 (rad)
        self.online: List[bool] = [False] * TOTAL_JOINTS
        self.last_diag = None                           # 最新一帧诊断, 打印时才细读

        # -- 窗口累积器: 每打一屏取走并清零 (见模块 docstring 的 interval 说明) --
        self.n_state = self.n_diag = 0                  # 收到的帧数 -> 换算实际帧率
        self._i_win: Dict[int, float] = {}              # nid -> 本窗口电流峰值
        self._t_win: Dict[int, float] = {}              # nid -> 本窗口温度峰值
        self._v_win = 99.0                              # 本窗口母线最低值
        self._lim_win: Dict[int, int] = {}              # nid -> 本窗口触发过的限幅掩码
        # 错误码存**集合**而不是按位或: 或出来的值不一定是个合法码, 解码就成了乱码。
        self._err_win: Dict[int, Set[int]] = {}         # nid -> 本窗口出现过的码

    # -- 连接 ---------------------------------------------------------------
    def connect(self, timeout_ms: int = 3000) -> None:
        self.hand = SdkManager.instance().connect(
            address=self.address, device_name=self.alias,
            # 一律 True: 本脚本天生是「第二个客户端」, 必须走桥。见模块 docstring。
            options=ConnectOptions(timeout_ms=timeout_ms, retry_count=3,
                                   enable_bridge=True))
        self.serial_number = self.hand.serial_number
        try:
            self.handedness = self.hand.handedness().get()
        except Exception:                                   # noqa: BLE001
            pass
        try:
            self.online_count = int(self.hand.online_joints_count().get())
        except Exception:                                   # noqa: BLE001
            pass
        # 回调订阅, 不用 subscribe()+recv(): 流是 ~970Hz 而我们 1s 才看一眼,
        # 在主循环里收会每拍溢缓冲(SDK 刷 `lagged N messages` 警告)。
        self._state_sub = self.hand.joint_states().subscribe_with_callback(self._on_state)
        self._diag_sub = self.hand.joint_diagnostics().subscribe_with_callback(self._on_diag)

    def close(self) -> None:
        for stream in (self._state_sub, self._diag_sub):
            if stream is not None:
                self._try(stream.close)
        if self.hand is not None:
            self._try(self.hand.disconnect)

    @staticmethod
    def _try(fn) -> None:
        try:
            fn()
        except Exception:                                   # noqa: BLE001
            pass

    # -- 回调 (跑在 SDK 后台线程, 只做赋值和累积, 不打印不解码) ---------------
    def _on_state(self, frame) -> None:
        for j in frame.joints:                  # 变长, 只含在线关节 -> 出现过 = 在线
            k = nid_to_flat(j.nid)
            if k >= 0:
                self.pos[k] = j.position
                self.online[k] = True
        self.n_state += 1

    def _on_diag(self, frame) -> None:
        for j in frame.joints:
            nid = j.nid
            i = abs(j.current)
            if i > self._i_win.get(nid, -1.0):
                self._i_win[nid] = i
            t = j.mcu_temp_c_fb
            if t > self._t_win.get(nid, -99.0):
                self._t_win[nid] = t
            if j.vbus_v_fb < self._v_win:
                self._v_win = j.vbus_v_fb
            sw = j.status_word
            mask = ((LIM_CUR if sw.current_limit_active else 0)
                    | (LIM_POS if sw.position_limit_active else 0)
                    | (LIM_VEL if sw.velocity_limit_active else 0))
            if mask:
                self._lim_win[nid] = self._lim_win.get(nid, 0) | mask
            if j.error_code_current:
                self._err_win.setdefault(nid, set()).add(j.error_code_current)
        self.last_diag = frame
        self.n_diag += 1

    # -- 取窗口 -------------------------------------------------------------
    def take_window(self) -> Tuple[int, int, Dict[int, float], Dict[int, float],
                                   float, Dict[int, int], Dict[int, Set[int]]]:
        """取走本窗口的累积值并清零。返回的都是快照, 之后回调再怎么写都不影响。"""
        n_s, self.n_state = self.n_state, 0
        n_d, self.n_diag = self.n_diag, 0
        i_win, self._i_win = self._i_win, {}
        t_win, self._t_win = self._t_win, {}
        v_win, self._v_win = self._v_win, 99.0
        lim, self._lim_win = self._lim_win, {}
        err, self._err_win = self._err_win, {}
        return n_s, n_d, i_win, t_win, v_win, lim, err


# --------------------------------------------------------------------------
# 渲染
# --------------------------------------------------------------------------

LEGEND = """表头: angle/deg=关节角(瞬时)  cur/A=电流  temp/C=温度  lim=限幅  err=错误码
      cur / temp / lim / err 四列都是**本窗口(上一屏到这一屏)内**的峰值或出现过的标志,
      所以 --interval 调粗也不会漏掉一闪而过的限流或错误码。
      lim: 流=限流(堵转或撞上了) 位=位置限位 速=速度限幅
      err 是固件原样上报的 u16, **含告警位, 非零不等于故障** -- 判不判停机交给看的人。"""


def _code_txt(c: int) -> str:
    d = describe_code(c)
    return "0x%04X(%s)" % (c, d) if d else "0x%04X" % c


def _codes_txt(codes: Set[int]) -> str:
    return ",".join(_code_txt(c) for c in sorted(codes))


def _lim_txt(mask: int) -> str:
    return "".join(ch for bit, ch in ((LIM_CUR, "流"), (LIM_POS, "位"), (LIM_VEL, "速"))
                   if mask & bit) or "-"


def _hot_joint(t_win: Dict[int, float]) -> Tuple[str, float]:
    if not t_win:
        return "?", 0.0
    nid, t = max(t_win.items(), key=lambda kv: kv[1])
    k = nid_to_flat(nid)
    return (joint_label(k) if k >= 0 else "nid%d" % nid), t


def render_full(w: HandWatch, dt: float) -> List[str]:
    n_s, n_d, i_win, t_win, v_win, lim, err = w.take_window()
    hz_s, hz_d = (n_s / dt, n_d / dt) if dt > 0 else (0.0, 0.0)
    out = ["[%s] SN=%s %s 在线 %d/%d   states %.0fHz  diag %.0fHz"
           % (w.alias, w.serial_number, w.handedness,
              sum(w.online), TOTAL_JOINTS, hz_s, hz_d)]
    frame = w.last_diag
    if n_d == 0 or frame is None:
        out.append("  !! 本窗口没收到诊断帧 -- 流停了(设备掉线, 或桥主进程退了)")
        return out

    out.append("  %-14s %-22s %9s %8s %8s  %-4s %s"
               % ("joint", "state", "angle/deg", "cur/A", "temp/C", "lim", "err"))
    for j in frame.joints:
        k = nid_to_flat(j.nid)
        if k < 0:                                   # 触觉节点, 无电机
            continue
        codes = err.get(j.nid)
        out.append("  %-14s %-22s %9.1f %8.3f %8.1f  %-4s %s"
                   % (joint_label(k), j.status_word.ext_state_name,
                      w.pos[k] * R2D,
                      i_win.get(j.nid, 0.0), t_win.get(j.nid, 0.0),
                      _lim_txt(lim.get(j.nid, 0)),
                      _codes_txt(codes) if codes else "-"))
    hot, hot_t = _hot_joint(t_win)
    out.append("  汇总: 母线最低 %.2fV  电流峰值 %.3fA  最高温度 %.1fC@%s"
               % (v_win, max(i_win.values()) if i_win else 0.0, hot_t, hot))
    return out


def render_compact(w: HandWatch, dt: float) -> List[str]:
    _n_s, n_d, i_win, t_win, v_win, lim, err = w.take_window()
    hz_d = n_d / dt if dt > 0 else 0.0
    if n_d == 0:
        return ["[%s] SN=%s  !! 本窗口无诊断帧 -- 流停了" % (w.alias, w.serial_number)]
    hot, hot_t = _hot_joint(t_win)
    txt = ("[%s] SN=%s %s %d/%d  diag %.0fHz  母线>=%.2fV  电流<=%.3fA  最热 %s %.1fC"
           % (w.alias, w.serial_number, w.handedness, sum(w.online), TOTAL_JOINTS,
              hz_d, v_win, max(i_win.values()) if i_win else 0.0, hot, hot_t))
    if lim:
        txt += "  限幅 " + ",".join(
            "%s[%s]" % (joint_label(nid_to_flat(n)), _lim_txt(m))
            for n, m in sorted(lim.items()) if nid_to_flat(n) >= 0)
    if err:
        txt += "  错误码 " + ",".join(
            "%s=%s" % (joint_label(nid_to_flat(n)), _codes_txt(c))
            for n, c in sorted(err.items()) if nid_to_flat(n) >= 0)
    return [txt]


# --------------------------------------------------------------------------
# 命令行
# --------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="只读监视 Wuji Hand 2 的实时数据(角度/温度/电流/母线/错误码)。"
                    "不使能、不下发、不改配置。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--address", action="append", metavar="IP:PORT",
                    help="设备地址, 可重复 = 同时看多只手。省略则自动扫描。端口通常是 7447")
    ap.add_argument("--interval", type=float, default=1.0, metavar="S",
                    help="屏幕刷新间隔(秒), 默认 1.0。**只影响刷屏, 不改设备推送速率**; "
                         "峰值/限幅/错误码按窗口累积, 调粗也不漏事件")
    ap.add_argument("--duration", default="0", metavar="T",
                    help="看多久自动退出, 支持 90s / 30m / 8h; 0 = 看到 Ctrl+C")
    ap.add_argument("--compact", action="store_true",
                    help="每只手只打一行汇总(手多时用这个, 20 行一只刷不过来)")
    ap.add_argument("--once", action="store_true",
                    help="打一屏就退出 -- 给自动化设备取快照用")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="打印 SDK 内部日志 (连不上时用来看细节)")
    return ap


def resolve_targets(addresses: Optional[List[str]]) -> List[Tuple[str, str]]:
    """-> [(alias, address), ...]; 出错时打印给操作员看的话并返回空表。"""
    if addresses:
        dup = sorted({a for i, a in enumerate(addresses) if a in addresses[:i]})
        if dup:
            print("!! --address 里有重复地址 %s -- 一只手会被当成两只看。请每只写一次。"
                  % ", ".join(dup))
            return []
        return [("hand%d" % (i + 1), a) for i, a in enumerate(addresses)]

    print("未指定 --address, 自动扫描 ...")
    found = discover_hands()
    if not found:
        print("!! 没扫到任何 Wuji Hand 2, 检查网线/供电, 或用 --address 手动指定")
        return []
    for sn, addr in found:
        print("  发现 %s @ %s" % (sn, addr))
    # 撞 IP: 同侧手出厂默认 IP 相同, 两只一起上电就都答同一个地址, 谁都连不上。
    by_addr: Dict[str, List[str]] = {}
    for sn, addr in found:
        by_addr.setdefault(addr, []).append(sn)
    clash = {a: sns for a, sns in by_addr.items() if len(sns) > 1}
    if clash:
        print("!! 撞 IP -- 下面这些地址被多只手同时占着, 谁都连不上:")
        for addr, sns in sorted(clash.items()):
            print("     %s  <-  %s" % (addr, ", ".join(sorted(sns))))
        print("   逐只改 IP 见 README 第 6 节 (set_hand_ip.py)。")
        return []
    return found


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    wuji_sdk.set_log_level("info" if args.verbose else "warn")

    interval = max(MIN_INTERVAL_S, args.interval)
    if interval != args.interval:
        print("注意: --interval %.3f 太密, 已收到下限 %.2fs" % (args.interval, interval))
    duration = parse_duration(args.duration)

    targets = resolve_targets(args.address)
    if not targets:
        return 2

    print("=== 只读监视 (不使能 / 不下发 / 不改配置) ===")
    print("刷新 %.2fs/屏%s; Ctrl+C 退出"
          % (interval, ", 打一屏就退" if args.once else ""))
    if len(targets) >= 3 and not args.compact:
        print("提示: %d 只手 x 20 行会刷不过来, 建议加 --compact" % len(targets))

    # 串行连接: SDK 的连接握手会把已连上的手的订阅回调饿死(实测报 `lagged N messages`),
    # 所以一只一只连, 连完再进打印循环。
    watches: List[HandWatch] = []
    for alias, address in targets:
        w = HandWatch(alias, address)
        print("[%s] 连接 %s ..." % (alias, address))
        try:
            w.connect()
        except Exception as exc:                            # noqa: BLE001
            print("[%s] 连接失败: %s%s" % (alias, exc, connect_hint(exc)))
            print("   本脚本是「第二个客户端」, 所以还要多看一条: **老化脚本必须带"
                  " --bridge 起**, 否则它独占设备那唯一一条会话, 本脚本无论如何都连不上。")
            continue
        print("[%s] 已连接 SN=%s 手型=%s 在线关节=%d/20"
              % (alias, w.serial_number, w.handedness, w.online_count))
        watches.append(w)

    if not watches:
        return 2

    print("\n[!] 顺序要求: 先起老化(带 --bridge, 当桥主), 再起本脚本; 退出时先退本脚本。")
    print("    本脚本先起的话它会成为桥主, 老化挂在它身上 -- 关掉本脚本老化会断链。")
    print("\n" + LEGEND + "\n")

    stop = [False]

    def on_sigint(_sig, _frm):
        stop[0] = True
        print("\n>> 收到 Ctrl+C, 断开监视(未对手做任何改动)")

    signal.signal(signal.SIGINT, on_sigint)

    # 先等第一帧: 刚订阅上就打印会是一屏空表, 看的人会以为流不通。
    t_wait = time.perf_counter()
    while (not stop[0] and any(w.n_diag == 0 for w in watches)
           and time.perf_counter() - t_wait < FIRST_FRAME_TIMEOUT_S):
        time.sleep(0.02)
    for w in watches:
        if w.n_diag == 0:
            print("[%s] !! %.0fs 内没收到 joint_diagnostics -- 反馈流本身不通"
                  % (w.alias, FIRST_FRAME_TIMEOUT_S))

    render = render_compact if args.compact else render_full
    t_last = t_wait
    deadline = (time.perf_counter() + duration) if duration > 0 else None
    try:
        while not stop[0]:
            now = time.perf_counter()
            dt, t_last = now - t_last, now
            print("---- %s ----" % time.strftime("%H:%M:%S"))
            for w in watches:
                for line in render(w, dt):
                    print(line)
            print("", flush=True)
            if args.once:
                break
            if deadline is not None and time.perf_counter() >= deadline:
                print(">> 到了 --duration 上限, 退出")
                break
            # 分片睡: Windows 上 time.sleep 长睡期间 Ctrl+C 要等醒来才响应, 分片能秒退。
            wake = now + interval
            while not stop[0] and time.perf_counter() < wake:
                time.sleep(min(0.2, max(0.0, wake - time.perf_counter())))
    finally:
        for w in watches:
            w.close()
        print("=== 监视结束, 未对手做任何改动 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
