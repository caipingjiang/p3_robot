#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P3DX 串口协议验证脚本（实机测试 · 阶段 1）

目的：不依赖 ROS2，先用 pyserial 直接跟底盘 ARCOS 固件对话，验证三件最不确定的事——
  1. 波特率（默认 9600）是否正确；
  2. P2OS/ARCOS 二进制协议实现是否正确（checksum、SYNC 握手）；
  3. 里程计数据流（标准 SIP 的 X/Y/Th/速度/电压）能否正常读到。

协议依据：《Pioneer 3 Operations Manual v5 (2007)》第 6 章 ARCOS。

用法：
  # 离线自测（不连串口，验证命令字节与手册一致）
  python3 p3dx_serial_probe.py --selftest

  # 实机验证（先只读 SIP，不使能电机，最安全）
  python3 p3dx_serial_probe.py --port /dev/ttyUSB0

  # 使能电机（确认 ENABLE 命令生效，成功后机器人常会“嘀嘀嘀”）
  python3 p3dx_serial_probe.py --port /dev/ttyUSB0 --enable

  # 使能电机 + 试走一小段（前 0.3 m/s 1.5 秒，验证运动）
  python3 p3dx_serial_probe.py --port /dev/ttyUSB0 --enable --test-move

依赖：pip install pyserial
"""

import argparse
import struct
import sys
import time

# ──────────────────────────────────────────────────────────────────────
# 协议常量（来自手册 Table 7 命令集）
# ──────────────────────────────────────────────────────────────────────
HEADER = b"\xfa\xfb"          # 所有包共用头 0xFA 0xFB
ARG_INT  = 0x3B               # 正整数参数类型
ARG_SINT = 0x1B               # 负/绝对整数参数类型

SYNC0, SYNC1, SYNC2 = 0, 1, 2
PULSE, OPEN, CLOSE, ENABLE = 0, 1, 2, 4
SETA, SETV, SETO = 5, 6, 7
VEL, RVEL, SETRV = 11, 21, 10
STOP = 29

# 标准 SIP 的 type 高半字节（0x30）；低半字节 2=停 / 3=动
SIP_STANDARD = 0x30

# THPOS 角度单位换算：2π/4096 弧度/单位
ANGLE_UNIT_RAD = 2.0 * 3.141592653589793 / 4096.0


# ──────────────────────────────────────────────────────────────────────
# checksum —— 照抄手册 ARIA ArRobotPacket::calcCheckSum()
# ──────────────────────────────────────────────────────────────────────
def calc_checksum(body: bytes) -> int:
    """对包的数据部分（从 command/type 字节起，不含 header 和 count，也不含
    checksum 自身）计算 16 位校验和：逐对字节相加（高字节在前）累加 &0xffff，
    奇数个字节时末字节 XOR 到低字节。"""
    c = 0
    i = 0
    n = len(body)
    while n > 1:
        c = (c + ((body[i] << 8) | body[i + 1])) & 0xFFFF
        n -= 2
        i += 2
    if n > 0:
        c ^= body[i]
    return c


def build_command(cmd: int, arg: int | None = None) -> bytes:
    """组装一条客户端命令包。

    cmd  = 命令号（Table 7）
    arg  = 整数参数（可选）。正数用 0x3B，负数用 0x1B（存其绝对值）。
           2 字节小端；checksum 2 字节大端放包尾。
    """
    body = bytearray([cmd])
    if arg is not None:
        body.append(ARG_INT if arg >= 0 else ARG_SINT)
        body += struct.pack("<H", arg & 0xFFFF)   # 小端，负数转无符号存储
    count = len(body) + 2                          # + 2 字节 checksum
    checksum = calc_checksum(bytes(body))
    return HEADER + bytes([count]) + bytes(body) + struct.pack(">H", checksum)


# ──────────────────────────────────────────────────────────────────────
# 串口包读取 —— 从字节流中同步到 0xFA 0xFB 头并读出一个完整包
# ──────────────────────────────────────────────────────────────────────
def read_packet(ser, timeout: float = 2.0) -> dict | None:
    """从串口读一个完整数据包。返回 dict{type, payload, checksum_ok, raw} 或
    超时返回 None。"""
    ser.timeout = timeout

    # 1) 同步到 header 0xFA 0xFB
    state = 0
    while True:
        b = ser.read(1)
        if not b:
            return None
        if state == 0:
            if b[0] == 0xFA:
                state = 1
        elif state == 1:
            if b[0] == 0xFB:
                break
            elif b[0] != 0xFA:
                state = 0

    # 2) 读 byte count
    cnt_b = ser.read(1)
    if not cnt_b:
        return None
    cnt = cnt_b[0]

    # 3) 读 type + payload + checksum（共 cnt 字节）
    data = ser.read(cnt)
    if len(data) < cnt:
        return None

    body = data[:-2]                               # type + payload
    checksum_recv = struct.unpack(">H", data[-2:])[0]
    return {
        "type": data[0],
        "payload": data[1:-2],
        "checksum_ok": checksum_recv == calc_checksum(body),
        "raw": data,
    }


def read_autoconfig(ser, timeout: float = 2.0) -> list[str]:
    """SYNC2 之后 ARCOS 回发 name / type / subtype 三个字符串。
    兼容两种编码：P2OS 长度前缀（1 字节长度 + 内容）与 NULL 结尾。"""
    ser.timeout = 0.2
    data = bytearray()
    deadline = time.time() + timeout
    last_rx = time.time()
    while time.time() < deadline:
        b = ser.read(1)
        if b:
            data += b
            last_rx = time.time()
        elif data and time.time() - last_rx > 0.3:   # 静默 0.3s 认为读完
            break
    if not data:
        return []

    # 尝试长度前缀：连读 3 个「1 字节长度 + N 字符」
    strings = []
    i = 0
    try:
        while len(strings) < 3 and i < len(data):
            n = data[i]
            i += 1
            strings.append(data[i:i + n].decode("ascii", errors="replace"))
            i += n
    except (IndexError, ValueError):
        strings = []
    if len(strings) == 3 and all(s.strip() for s in strings):
        return strings

    # 退回 NULL 结尾
    parts = [p.decode("ascii", errors="replace").strip()
             for p in bytes(data).split(b"\x00")]
    return [p for p in parts if p][:3]


# ──────────────────────────────────────────────────────────────────────
# 标准 SIP 解析 —— 只取里程计/速度/电压这几个驱动需要的字段
# ──────────────────────────────────────────────────────────────────────
def parse_standard_sip(pkt: dict) -> dict | None:
    if pkt["type"] & 0xF0 != SIP_STANDARD:
        return None
    p = pkt["payload"]
    # XPOS, YPOS, THPOS, LVEL, RVEL, BATTERY
    if len(p) < 2 + 2 + 2 + 2 + 2 + 1:
        return None
    x, y = struct.unpack("<hh", p[0:4])
    th = struct.unpack("<h", p[4:6])[0]
    lvel, rvel = struct.unpack("<hh", p[6:10])
    battery = p[10]                                  # 0.1 V
    return {
        "x_mm": x, "y_mm": y,
        "th_rad": th * ANGLE_UNIT_RAD,
        "th_deg": th * ANGLE_UNIT_RAD * 57.29577951308232,
        "lvel_mms": lvel, "rvel_mms": rvel,
        "battery_v": battery / 10.0,
        "moving": (pkt["type"] & 0x0F) == 3,
    }


def fmt_sip(s) -> str:
    return (f"X={s['x_mm']:+7d}mm  Y={s['y_mm']:+7d}mm  "
            f"Th={s['th_deg']:+7.2f}°  "
            f"L={s['lvel_mms']:+5d} R={s['rvel_mms']:+5d} mm/s  "
            f"电池={s['battery_v']:5.1f}V  {'动' if s['moving'] else '停'}")


# ──────────────────────────────────────────────────────────────────────
# 离线自测 —— 不连串口，验证命令字节与手册逐字节一致
# ──────────────────────────────────────────────────────────────────────
def selftest():
    cases = [
        ("SYNC0 250 251 3 0 0 0", build_command(SYNC0), b"\xfa\xfb\x03\x00\x00\x00"),
        ("SYNC1 250 251 3 1 0 1", build_command(SYNC1), b"\xfa\xfb\x03\x01\x00\x01"),
        ("SYNC2 250 251 3 2 0 2", build_command(SYNC2), b"\xfa\xfb\x03\x02\x00\x02"),
        ("OPEN  #1 250 251 3 1 0 1", build_command(OPEN), b"\xfa\xfb\x03\x01\x00\x01"),
        ("CLOSE #2 250 251 3 2 0 2", build_command(CLOSE), b"\xfa\xfb\x03\x02\x00\x02"),
        ("PULSE #0 250 251 3 0 0 0", build_command(PULSE), b"\xfa\xfb\x03\x00\x00\x00"),
        ("ENABLE #4 arg=1", build_command(ENABLE, 1), None),
        ("ENABLE #4 arg=0", build_command(ENABLE, 0), None),
        ("SETV #6 arg=500", build_command(SETV, 500), None),
        ("SETRV #10 arg=60", build_command(SETRV, 60), None),
        ("SETA #5 arg=300", build_command(SETA, 300), None),
        ("VEL #11 arg=+300", build_command(VEL, 300), None),
        ("VEL #11 arg=-300", build_command(VEL, -300), None),
        ("RVEL #21 arg=-60", build_command(RVEL, -60), None),
        ("STOP #29", build_command(STOP), None),
    ]
    ok = True
    for name, got, expect in cases:
        if expect is not None:
            good = got == expect
            if not good:
                ok = False
            print(f"[{'OK' if good else 'FAIL'}] {name}")
            print(f"       生成: {got.hex(' ')}")
            print(f"       期望: {expect.hex(' ')}")
        else:
            # 无手册期望值的命令，打印字节 + 校验 checksum 自洽
            body = got[3:-2]                       # command + argtype + arg
            recompute = calc_checksum(bytes(body))
            stored = struct.unpack(">H", got[-2:])[0]
            good = recompute == stored
            if not good:
                ok = False
            print(f"[{'OK' if good else 'FAIL'}] {name}")
            print(f"       字节: {got.hex(' ')}  (checksum 自洽={good})")
    print()
    print("全部通过 ✅" if ok else "存在失败 ❌")
    return ok


# ──────────────────────────────────────────────────────────────────────
# 主流程
# ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="P3DX 串口协议验证脚本")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--selftest", action="store_true", help="离线自测命令字节")
    ap.add_argument("--enable", action="store_true", help="连接后使能电机")
    ap.add_argument("--test-move", action="store_true",
                    help="使能后试走一小段（前进 0.3m/s 共 1.5s）")
    ap.add_argument("--debug", action="store_true",
                    help="打印每个包读到的原始字节（hex），用于诊断")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先安装：  pip install pyserial", file=sys.stderr)
        sys.exit(1)

    print(f"打开串口 {args.port} @ {args.baud} 8N1 ...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)

    # 打开串口会触发底盘 reset（CP210x 的 DTR 跳变），等它稳定并把 reset 垃圾冲掉
    print("等待底盘 reset 稳定 ...")
    time.sleep(3.0)
    ser.reset_input_buffer()

    # ---- 1) SYNC 握手（每步等回声再发下一步，带重试）----
    print("SYNC 握手 ...")
    for name, cmd in (("SYNC0", SYNC0), ("SYNC1", SYNC1), ("SYNC2", SYNC2)):
        echo = None
        for attempt in range(3):
            ser.reset_input_buffer()
            ser.write(build_command(cmd))
            echo = read_packet(ser, timeout=1.5)
            if args.debug and echo is not None:
                print(f"    [debug {name}#{attempt}] type={echo['type']:#04x} "
                      f"raw={echo['raw'].hex(' ')} checksum_ok={echo['checksum_ok']}")
            if echo is not None and echo["type"] == cmd and echo["checksum_ok"]:
                break
            echo = None
            time.sleep(0.5)
        if echo is None:
            print(f"  ✗ {name} 3 次尝试均无正确回声")
            print("    请确认：底盘已上电、串口线插对、且上一次连接已正常断开")
            print("    可加 --debug 查看底盘实际返回的字节")
            ser.close()
            sys.exit(1)
        print(f"  ✓ {name} 回声正常 (checksum_ok={echo['checksum_ok']})")

    # ---- 2) SYNC2 后读取机器人身份 ----
    ident = read_autoconfig(ser)
    if ident:
        print(f"  机器人身份: name={ident[0]!r}  type={ident[1]!r}  subtype={ident[2]!r}")
    else:
        print("  (未读到 name/type/subtype，继续尝试 OPEN)")

    # ---- 3) OPEN 启动服务器 ----
    ser.write(build_command(OPEN))
    print("已发送 OPEN #1，等待 SIP 数据流 ...")

    # ---- 4) 可选：使能电机 ----
    if args.enable:
        ser.write(build_command(ENABLE, 1))
        print("已发送 ENABLE #4 (arg=1)。若听到“嘀嘀嘀”说明电机已使能。")

    # ---- 5) 可选：试走一小段 ----
    if args.test_move:
        if not args.enable:
            print("⚠ --test-move 需要同时加 --enable，跳过试走")
        else:
            # 先设温和的速度/加速度上限，再前进
            ser.write(build_command(SETA, 200))     # 平移加速度 200 mm/s²
            ser.write(build_command(SETV, 300))     # 最大平移 300 mm/s
            ser.write(build_command(SETRV, 60))     # 最大旋转 60 deg/s
            ser.write(build_command(VEL, 300))      # 前进 0.3 m/s
            print("已发送 VEL #11 (0.3 m/s 前进)，1.5s 后停止 ...")
            time.sleep(1.5)
            ser.write(build_command(STOP))
            print("已发送 STOP #29")

    # ---- 6) 主循环：打印 SIP，并每 2 秒喂一次看门狗 ----
    print()
    print("读取 SIP 中（Ctrl+C 退出）。喂狗间隔 2s。")
    last_pulse = time.time()
    try:
        while True:
            pkt = read_packet(ser, timeout=1.0)
            if pkt is None:
                continue
            if not pkt["checksum_ok"]:
                print(f"  ⚠ 收到 checksum 错误包 type={pkt['type']:#x}，已忽略")
                continue
            sip = parse_standard_sip(pkt)
            if sip is not None:
                print(f"\r{fmt_sip(sip)}", end="", flush=True)
            now = time.time()
            if now - last_pulse >= 2.0:
                ser.write(build_command(PULSE))
                last_pulse = now
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全退出 ...")
    finally:
        try:
            ser.write(build_command(STOP))          # 若在动，先停
            ser.write(build_command(CLOSE))         # 关闭连接（自动禁用电机）
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print("已发送 STOP + CLOSE，串口已关闭。完成。")


if __name__ == "__main__":
    main()
