#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P3DX 波特率自动扫描（诊断用）

当 SYNC0 无回声时，自动尝试一组候选波特率，找出底盘实际在用的那个。
原理：ARCOS 处于 wait state 时会原样回声 SYNC 包，所以只要波特率匹配，
发 SYNC0 必然能读到 0xFA 0xFB 03 00 的回声。

用法：
  python3 p3dx_scan_baud.py --port /dev/ttyUSB0
"""

import argparse
import sys
import time


def try_baud(port, baud):
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        return False, f"打开失败: {e}"
    try:
        ser.reset_input_buffer()
        ser.write(bytes([0xFA, 0xFB, 0x03, 0x00, 0x00, 0x00]))   # SYNC0
        # 读回声：期望 fa fb 03 00 00 00（头 + count + cmd + checksum）
        data = bytearray()
        deadline = time.time() + 0.8
        while time.time() < deadline and len(data) < 6:
            b = ser.read(1)
            if b:
                data += b
        # 只要收到 fa fb 头就认为波特率命中（ARCOS 会回声整个包）
        return bytes(data).startswith(b"\xfa\xfb"), data.hex(" ") or "(无数据)"
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("缺少 pyserial：pip install pyserial", file=sys.stderr)
        sys.exit(1)

    # 候选波特率：默认 9600 优先，其余按可能性排列
    candidates = [9600, 19200, 38400, 57600, 115200, 4800]
    print(f"扫描串口 {args.port}，尝试波特率 ...\n")
    hit = None
    for baud in candidates:
        ok, info = try_baud(args.port, baud)
        mark = "✅ 命中" if ok else "  ✗"
        print(f"{mark}  {baud:6d} bps   回声: {info}")
        if ok:
            hit = baud
    print()
    if hit:
        print(f"→ 底盘实际波特率是 {hit} bps。用它重跑验证脚本：")
        print(f"  python3 src/p3dx_base/scripts/p3dx_serial_probe.py --port {args.port} --baud {hit}")
    else:
        print("→ 所有候选波特率都无回声。这不是波特率问题，请按顺序排查：")
        print("  1) 底盘是否上电？(机身 Main Power 开关拨到 ON)")
        print("  2) 串口线插的是底盘哪个口？(SERIAL 9针口 或 HOST 口，换一个试试)")
        print("  3) 串口线本身是否完好 / 是否直连对 TX-RX")
        print("  4) 底盘是否卡在 maintenance 模式（此时波特率固定 9600，需重上电恢复）")


if __name__ == "__main__":
    main()
