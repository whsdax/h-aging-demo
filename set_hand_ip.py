#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修改灵巧手 IP —— 一台电脑 + 一台交换机同时接多只手时用。

**为什么需要**: 出厂默认 IP 按左右手写死 —— 所有左手都是 `192.168.1.110`,
所有右手都是 `192.168.1.111`。同侧两只手一起上电就会**撞 IP**, 谁都连不上。
所以同侧的第 2/3/4 只手要各改一个空闲的唯一 IP (推荐 `.112` / `.113` / `.114`),
之后才能一拖多各连各的。

    1 左 1 右  → 默认 IP 就不撞, 不用改
    2 左 2 右  → 左2 改 .112, 右2 改 .113
    4 只全左手 → 3 只分别改 .112 / .113 / .114

用法:
    python set_hand_ip.py --show                    # 只看当前 IP/SN/手型, 不改
    python set_hand_ip.py --ip 192.168.1.112        # 改成指定 IP
    python set_hand_ip.py --reset-default           # 按手型复位成出厂默认
    python set_hand_ip.py --address 192.168.1.112:7447 --reset-default

🔴 **改 IP 时一次只上电一只手。** 同侧默认 IP 相同, 同时上电会撞车。
   一只配完 → 断电 → 再上电下一只。

🔴 **手离开工站 / 发货前必须 `--reset-default` 复位**, 否则它带着 .112 出去会影响别处。

原理: `ip().set()` 把新 IP 写进设备 flash(掉电不丢), 但**不立即生效** ——
固件是 deferred-apply, 必须重启才切换。所以 SET 之后、reboot 之前 GET 读到的
仍然是旧 IP, 这不是 bug。重启约需 40 秒。
"""

from __future__ import annotations

import argparse
import sys
import time

import wuji_sdk
from wuji_sdk import ConnectOptions, SdkManager

from aging_common import connect_hint      # 连接失败的人话提示, 和老化脚本共用一份

DEFAULT_PORT = 7447
GATEWAY_OCTET = 1              # 固件把子网/网关写死成 /24 + .1
RECOMMENDED = ("192.168.1.112", "192.168.1.113", "192.168.1.114")
DEFAULT_IP = {"left": "192.168.1.110", "right": "192.168.1.111"}
REBOOT_WAIT_S = 60             # 手重启约 40s, 留足余量
RECONNECT_INTERVAL_S = 3.0


# ──────────────────────────────────────────────────────────────────────────
# 小工具
# ──────────────────────────────────────────────────────────────────────────

def split_addr(addr: str):
    """'192.168.1.110:7447' → ('192.168.1.110', 7447)。"""
    if ":" in addr:
        ip, port = addr.rsplit(":", 1)
        return ip, int(port)
    return addr, DEFAULT_PORT


def validate_ip(target: str, current: str) -> None:
    """客户端预检 —— 设备侧也会校验并拒绝非法地址, 但早点报错省一次重启。"""
    parts = target.split(".")
    if len(parts) != 4 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
        raise ValueError("不是合法的 IPv4 地址: %s" % target)
    last = int(parts[3])
    if last in (0, 255):
        raise ValueError("%s 是网络地址/广播地址, 不能用" % target)
    if last == GATEWAY_OCTET:
        raise ValueError("%s 是网关地址, 不能用" % target)
    # 固件网段写死 /24, 跨网段改完会直接失联(且只能拆机或复位才能救回来)。
    if parts[:3] != current.split(".")[:3]:
        raise ValueError("目标 IP 必须和当前 IP 在同一 /24 网段 (当前 %s), "
                         "固件的子网/网关是写死的" % current)


def connect(address: str, name: str, timeout_ms: int = 3000):
    return SdkManager.instance().connect(
        address=address, device_name=name,
        options=ConnectOptions(timeout_ms=timeout_ms, retry_count=3, enable_bridge=True))


def find_single_hand(explicit: str = None) -> str:
    """定位要改的那只手。没显式指定就扫描, 扫到多只直接拒绝。"""
    if explicit:
        return explicit
    found = [(d.sn, d.address) for d in SdkManager.instance().scan()
             if d.device_type == wuji_sdk.DeviceType.WujiHand2]
    if not found:
        raise RuntimeError("没扫到手。手上电后要约 40 秒才就绪, 多等一会再试; "
                           "也可以用 --address 手动指定。")
    if len(found) > 1:
        raise RuntimeError(
            "扫到 %d 只手, 改 IP 时必须一次只上电一只 —— 同侧默认 IP 相同, "
            "同时上电会撞车。请断电其余的手, 或用 --address 显式指定。\n  %s"
            % (len(found), "\n  ".join("%s @ %s" % f for f in found)))
    sn, addr = found[0]
    print("扫到 %s @ %s" % (sn, addr))
    return addr


def hand_default_ip(hand) -> str:
    """出厂默认 IP: 优先按设备上报的手型, 读不到就退回 SN 第 4 位 (J=左 K=右)。"""
    try:
        side = str(hand.handedness().get()).lower()
        if side in DEFAULT_IP:
            return DEFAULT_IP[side]
    except Exception:                                     # noqa: BLE001
        pass
    sn = hand.serial_number or ""
    if len(sn) > 3 and sn[3].upper() in ("J", "K"):
        return DEFAULT_IP["left" if sn[3].upper() == "J" else "right"]
    raise RuntimeError("判不出手型 (handedness 读不到且 SN 第 4 位不是 J/K), "
                       "无法确定出厂默认 IP。请用 --ip 显式指定。")


def check_occupied(target_ip: str, self_sn: str) -> None:
    """扫一遍网段, 确认目标 IP 没被**别的**手占着。

    两只手撞同一个 IP 之后要断电逐只排查, 比这里多花 2 秒扫描贵得多。
    """
    for d in SdkManager.instance().scan():
        ip, _ = split_addr(d.address)
        if ip == target_ip and d.sn != self_sn:
            raise RuntimeError("目标 IP %s 已被 %s 占用, 换一个 (推荐 %s)"
                               % (target_ip, d.sn, " / ".join(RECOMMENDED)))


# ──────────────────────────────────────────────────────────────────────────
# 主流程
# ──────────────────────────────────────────────────────────────────────────

def change_ip(address: str, target_ip: str, assume_yes: bool) -> int:
    ip_now, port = split_addr(address)
    print("连接 %s ..." % address)
    hand = connect(address, "ipcfg")
    sn = hand.serial_number
    side = hand.handedness().get()
    live_ip = hand.ip().get()
    print("已连接 SN=%s 手型=%s 当前 IP=%s" % (sn, side, live_ip))

    validate_ip(target_ip, live_ip)
    if target_ip == live_ip:
        print("当前 IP 已经是 %s, 无需修改。" % target_ip)
        hand.disconnect()
        return 0
    if target_ip not in RECOMMENDED and target_ip not in DEFAULT_IP.values():
        print("提示: %s 不在推荐列表 (%s) 内, 请自行确认没和别的设备冲突。"
              % (target_ip, " / ".join(RECOMMENDED)))

    if not assume_yes:
        ans = input("确认把 %s 的 IP 从 %s 改成 %s? 设备会重启, 约 40 秒。[y/N] "
                    % (sn, live_ip, target_ip)).strip().lower()
        if ans not in ("y", "yes"):
            print("已取消。")
            hand.disconnect()
            return 1

    print("[0/4] 扫描网段检查目标 IP 是否被占用 ...")
    check_occupied(target_ip, sn)

    print("[1/4] 写入 IP 到设备 flash ...")
    hand.ip().set(target_ip)      # deferred-apply: 写 flash, 重启才生效

    print("[2/4] 触发设备重启 ...")
    try:
        hand.reboot()
    except Exception as exc:                              # noqa: BLE001
        # 设备重启时连接立刻断掉, 这里报错是正常现象, 不代表没重启成功。
        print("      (重启命令返回异常, 通常是连接被设备主动断开: %s)" % exc)
    try:
        hand.disconnect()
    except Exception:                                     # noqa: BLE001
        pass

    new_addr = "%s:%d" % (target_ip, port)
    print("[3/4] 连接新地址 %s ...(最多 %d 秒)" % (new_addr, REBOOT_WAIT_S))
    hand2 = wait_connect(new_addr, "ipcfg_new", REBOOT_WAIT_S)
    if hand2 is None:
        print("!! 超时连不上新地址。手可能重启偏慢 —— 再等一会手动确认:")
        print("     python set_hand_ip.py --address %s --show" % new_addr)
        return 2

    print("[4/4] 读回 IP 校验 ...")
    got = hand2.ip().get()
    ok = (got == target_ip)
    print("[%s] 设备 IP %s= %s (SN=%s)"
          % ("成功" if ok else "失败", "已生效 " if ok else "读回 ", got, hand2.serial_number))
    if ok and target_ip not in DEFAULT_IP.values():
        print(">> 记得: 这只手离开工站 / 发货前要跑 "
              "`python set_hand_ip.py --address %s --reset-default` 复位。" % new_addr)
    hand2.disconnect()
    return 0 if ok else 2


def wait_connect(address: str, name: str, timeout_s: float):
    """轮询连接, 直到设备重启完成。返回 hand 或 None。"""
    deadline = time.perf_counter() + timeout_s
    attempt = 0
    while time.perf_counter() < deadline:
        attempt += 1
        try:
            return connect(address, "%s%d" % (name, attempt))
        except Exception:                                 # noqa: BLE001
            left = deadline - time.perf_counter()
            print("      第 %d 次未连上, 剩余 %.0fs ..." % (attempt, max(0.0, left)))
            time.sleep(RECONNECT_INTERVAL_S)
    return None


def show(address: str) -> int:
    hand = connect(address, "ipshow")
    live = hand.ip().get()
    default = None
    try:
        default = hand_default_ip(hand)
    except RuntimeError:
        pass
    print("SN        : %s" % hand.serial_number)
    print("手型      : %s" % hand.handedness().get())
    print("当前 IP   : %s" % live)
    print("出厂默认  : %s" % (default or "判不出"))
    print("在线关节  : %d/20" % hand.online_joints_count().get())
    if default and live != default:
        print(">> 本手 IP 不是出厂默认。若要下线/交付, 用 --reset-default 复位。")
    hand.disconnect()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""🔴 改 IP 时一次只上电一只手 —— 同侧默认 IP 相同, 同时上电会撞车。
🔴 手离开工站 / 发货前必须 --reset-default 复位。

出厂默认: 左手 192.168.1.110 / 右手 192.168.1.111
推荐可用: 192.168.1.112 / .113 / .114
电脑网卡: 192.168.1.100 / 255.255.255.0 / 网关 192.168.1.1
""")
    ap.add_argument("--address", metavar="IP:PORT",
                    help="手的当前地址; 省略则自动扫描(要求当前只有一只手在线)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--ip", metavar="IP", help="目标 IP (推荐 192.168.1.112/.113/.114)")
    g.add_argument("--reset-default", action="store_true",
                   help="复位成出厂默认 (按手型自动选 .110/.111)")
    g.add_argument("--show", action="store_true", help="只显示当前 IP/SN/手型, 不修改")
    ap.add_argument("--yes", "-y", action="store_true", help="跳过确认提示")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="打印 SDK 内部日志 (默认不打 —— 重启期间的连接失败日志很吵)")
    args = ap.parse_args(argv)

    wuji_sdk.set_log_level("info" if args.verbose else "error")
    try:
        address = find_single_hand(args.address)
        if args.show:
            return show(address)
        target = args.ip
        if args.reset_default:
            hand = connect(address, "ipdefault")
            target = hand_default_ip(hand)
            print("按手型判定的出厂默认 IP = %s" % target)
            hand.disconnect()
            time.sleep(1.0)          # 让上一条连接彻底释放, 免得下面重连撞上
        return change_ip(address, target, args.yes)
    except (RuntimeError, ValueError) as exc:
        print("!! %s" % exc)          # 给操作员看的错, 不甩 traceback
        return 2
    except wuji_sdk.WujiException as exc:
        # SDK 的连接失败也别甩 traceback —— 那串 Rust 源码路径对操作员毫无用处。
        print("!! %s%s" % (exc, connect_hint(exc)))
        return 2
    except KeyboardInterrupt:
        print("\n已中断。注意: 若已走到 [1/4] 之后, IP 可能已写入, 用 --show 确认。")
        return 130


if __name__ == "__main__":
    sys.exit(main())
