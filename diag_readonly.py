#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读诊断 —— 查「使能了但不动、电流 0.00A」。

**不使能、不下发、不动手。** 只连接 + 回读参数 + 抓一帧 joint_states /
joint_diagnostics, 打印每个关节的 ext_state / 错误码 / 母线 / 电流 / 温度。

要回答的三个问题:
  1. 增益到底写进去了吗 (mit_params / effort_limit 回读值 vs 期望 5.0/0.1/1.0)
  2. 关节现在是什么状态 (ext_state —— 是 MIT 运行, 还是 off/fault)
  3. 母线到底多少 (静态值; 老化里那个「最低 4.7V」是不是塌过)
"""

from __future__ import annotations

import sys
import time

import wuji_sdk
from wuji_sdk import ConnectOptions, SdkManager

sys.path.insert(0, ".")
from aging_common import TOTAL_JOINTS, joint_label, nid_to_flat  # noqa: E402

EXPECT_KP, EXPECT_KD, EXPECT_IQ = 5.0, 0.1, 1.0


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    address = argv[0] if argv else "192.168.1.111:7447"

    print("=== 只读诊断 (不使能 / 不下发 / 不动手) ===")
    print("连接 %s ..." % address)
    hand = SdkManager.instance().connect(
        address=address, device_name="diag",
        options=ConnectOptions(timeout_ms=3000, retry_count=3, enable_bridge=False))
    print("已连接 SN=%s 手型=%s 在线关节=%s/20"
          % (hand.serial_number, hand.handedness().get(),
             hand.online_joints_count().get()))

    # ── 1. 回读控制参数 ──────────────────────────────────────────────────
    print("\n--- 1. 控制参数回读 (期望 kp=%.1f kd=%.2f iq=%.1fA) ---"
          % (EXPECT_KP, EXPECT_KD, EXPECT_IQ))
    try:
        mp = hand.mit_params().get()
        print("mit_params 回读: %r" % (mp,))
    except Exception as exc:                                  # noqa: BLE001
        print("mit_params 回读失败: %s" % exc)
    try:
        el = hand.effort_limit().get()
        js = list(getattr(el, "joints", []) or [])
        if js:
            print("effort_limit 回读: min=%.3fA max=%.3fA  全 20 轴=%s"
                  % (min(js), max(js), ["%.2f" % v for v in js]))
        else:
            print("effort_limit 回读: %r" % (el,))
    except Exception as exc:                                  # noqa: BLE001
        print("effort_limit 回读失败: %s" % exc)

    # ── 2. 抓诊断帧 ──────────────────────────────────────────────────────
    print("\n--- 2. 关节状态 / 母线 / 电流 / 温度 (静态, 未使能) ---")
    box = {}
    sub_d = hand.joint_diagnostics().subscribe_with_callback(
        lambda f: box.__setitem__("diag", f))
    sub_s = hand.joint_states().subscribe_with_callback(
        lambda f: box.__setitem__("state", f))
    t0 = time.perf_counter()
    while ("diag" not in box or "state" not in box) and time.perf_counter() - t0 < 5.0:
        time.sleep(0.02)

    if "diag" not in box:
        print("!! 5s 内没收到 joint_diagnostics —— 反馈流本身就不通")
    else:
        frame = box["diag"]
        print("%-14s %-22s %8s %8s %8s  %s"
              % ("joint", "ext_state", "vbus/V", "cur/A", "temp/C", "err"))
        vmin, imax = 99.0, 0.0
        states = {}
        for j in frame.joints:
            k = nid_to_flat(j.nid)
            if k < 0:
                continue
            sw = j.status_word
            name = sw.ext_state_name
            states[name] = states.get(name, 0) + 1
            vmin = min(vmin, j.vbus_v_fb)
            imax = max(imax, abs(j.current))
            flags = "".join(c for c, on in (
                ("流", sw.current_limit_active), ("位", sw.position_limit_active),
                ("速", sw.velocity_limit_active)) if on)
            err = ("0x%04X" % j.error_code_current) if j.error_code_current else "-"
            print("%-14s %-22s %8.2f %8.3f %8.1f  %s%s"
                  % (joint_label(k), name, j.vbus_v_fb, j.current,
                     j.mcu_temp_c_fb, err, (" [" + flags + "]") if flags else ""))
        print("\n汇总: 母线最低 %.2fV, 电流最大 %.3fA" % (vmin, imax))
        print("ext_state 分布: %s" % ", ".join("%s×%d" % kv for kv in sorted(states.items())))

    if "state" in box:
        pos = list(box["state"].position)
        print("\n当前关节角(度, 关节侧): %s"
              % ", ".join("%s=%.1f" % (joint_label(k), pos[k] * 57.2958)
                          for k in range(min(TOTAL_JOINTS, len(pos)))))

    del sub_d, sub_s
    print("\n=== 诊断结束, 未对手做任何改动 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
