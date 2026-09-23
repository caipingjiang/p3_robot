# -*- coding: utf-8 -*-
"""P3DX 底盘 ARCOS 串口协议驱动层（与 ROS 无关，可独立测试）。

封装了《Pioneer 3 Operations Manual v5》第 6 章 ARCOS 协议的：
  - checksum 计算、命令包组装、数据包读取
  - SYNC 握手 + OPEN + 身份信息解析
  - 运动命令（VEL/RVEL）、看门狗（PULSE）、电机使能（ENABLE）
  - 标准 SIP 解析（里程计 X/Y/Th、轮速、电池）

只依赖 pyserial，不依赖 ROS。被 ROS2 节点 p3dx_base_node 调用，
也被 scripts/p3dx_serial_probe.py 复用（未来可统一到这里）。
"""

import struct
import time

import serial

# ── 协议常量（手册 Table 7 命令集）──────────────────────────────────
HEADER = b"\xfa\xfb"
ARG_INT = 0x3B          # 正整数参数类型
ARG_SINT = 0x1B         # 负/绝对整数参数类型

SYNC0, SYNC1, SYNC2 = 0, 1, 2
PULSE, OPEN, CLOSE, ENABLE = 0, 1, 2, 4
SETA, SETV, SETO = 5, 6, 7
SETRV, VEL, RVEL = 10, 11, 21
SETRA = 23
STOP = 29

SIP_STANDARD = 0x30                      # 标准 SIP 的 type 高半字节
ANGLE_UNIT_RAD = 2.0 * 3.141592653589793 / 4096.0   # Thpos 单位换算


def calc_checksum(body: bytes) -> int:
    """对包数据部分（从 command/type 起，不含 header、count、checksum）算校验和。"""
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
    """组装一条客户端命令包（整数参数 2 字节小端，checksum 2 字节大端）。

    负数按 ARCOS 的 sign-magnitude 编码：类型字节 0x1B + 绝对值。
    不能存二补码——固件读到 0x1B 会把值取负，二补码取负后溢出翻转成正数，
    导致后退/右转全部变成前进/左转（详见 p3dx-serial-probe selftest）。
    """
    body = bytearray([cmd])
    if arg is not None:
        body.append(ARG_INT if arg >= 0 else ARG_SINT)
        body += struct.pack("<H", abs(arg))
    count = len(body) + 2
    return HEADER + bytes([count]) + bytes(body) + struct.pack(">H", calc_checksum(bytes(body)))


def parse_standard_sip(pkt: dict) -> dict | None:
    """从 read_packet 的结果解析标准 SIP 的里程计/速度/电池字段。"""
    if pkt["type"] & 0xF0 != SIP_STANDARD:
        return None
    p = pkt["payload"]
    # XPOS, YPOS, THPOS, LVEL, RVEL, BATTERY
    if len(p) < 2 + 2 + 2 + 2 + 2 + 1:
        return None
    x, y = struct.unpack("<hh", p[0:4])
    th = struct.unpack("<h", p[4:6])[0]
    lvel, rvel = struct.unpack("<hh", p[6:10])
    battery = p[10]
    return {
        "x_mm": x, "y_mm": y,
        "th_rad": th * ANGLE_UNIT_RAD,
        "lvel_mms": lvel, "rvel_mms": rvel,
        "battery_v": battery / 10.0,
        "moving": (pkt["type"] & 0x0F) == 3,
    }


class P3dxDriver:
    """P3DX 底盘串口驱动。用法：
        d = P3dxDriver('/dev/p3dx', 9600)
        d.connect()          # 握手 + OPEN
        d.enable_motors(True)
        d.set_velocity(300, 0)   # 前进 0.3 m/s
        sip = d.read_sip()   # 读里程计
        d.close()
    """

    def __init__(self, port: str, baud: int = 9600,
                 max_v_mms: int = 500, max_w_degs: int = 60,
                 accel_mms2: int = 300, accel_degs2: int = 100):
        self.port = port
        self.baud = baud
        self.max_v_mms = max_v_mms
        self.max_w_degs = max_w_degs
        self.accel_mms2 = accel_mms2
        self.accel_degs2 = accel_degs2
        self.ser = None
        # 身份信息（SYNC2 后从底盘读到）
        self.name = ""
        self.robot_type = ""
        self.subtype = ""
        self._connected = False
        self._limits_sent = False

    # ── 连接 ─────────────────────────────────────────────────────────
    def connect(self, sync_retries: int = 3) -> bool:
        """打开串口 + SYNC0/1/2 握手 + OPEN。成功返回 True。

        若底盘残留 OPEN 状态（上次连接未 CLOSE 就被强杀），第一次握手会失败；
        此时先发 CLOSE 让它回到 wait state，再重试一轮。
        """
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        # 打开串口会触发底盘 reset，等它稳定并冲掉 reset 垃圾数据
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        for attempt in range(2):
            if self._handshake(sync_retries):
                break
            # 握手失败：底盘可能卡在 OPEN 状态（持续发 SIP），发 CLOSE 重置后重试
            self.ser.reset_input_buffer()
            self.ser.write(build_command(CLOSE))
            time.sleep(1.0)
            self.ser.reset_input_buffer()
        else:
            self.ser.close()
            self.ser = None
            raise ConnectionError(
                f"握手失败（2 轮 × {sync_retries} 次重试）——检查端口/波特率/上电")

        self.ser.write(build_command(OPEN))
        self._connected = True
        return True

    def _handshake(self, sync_retries: int) -> bool:
        """执行 SYNC0/1/2 三步握手，全部成功返回 True。"""
        for name, cmd in (("SYNC0", SYNC0), ("SYNC1", SYNC1), ("SYNC2", SYNC2)):
            echo = self._sync_step(cmd, sync_retries)
            if echo is None:
                return False
            if cmd == SYNC2:
                self._parse_identity(echo)
        return True

    def _sync_step(self, cmd: int, retries: int) -> dict | None:
        """发一个 SYNC 命令并等正确回声（带重试）。"""
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(build_command(cmd))
            echo = self._read_packet(timeout=1.5)
            if echo is not None and echo["type"] == cmd and echo["checksum_ok"]:
                return echo
            time.sleep(0.4)
        return None

    def _parse_identity(self, echo: dict) -> None:
        """SYNC2 回声包的 payload = name\\0 + type\\0 + subtype\\0。"""
        parts = echo["payload"].split(b"\x00")
        if len(parts) >= 3:
            self.name = parts[0].decode("ascii", errors="replace")
            self.robot_type = parts[1].decode("ascii", errors="replace")
            self.subtype = parts[2].decode("ascii", errors="replace")

    # ── 底层收发 ─────────────────────────────────────────────────────
    def _read_packet(self, timeout: float = 0.5) -> dict | None:
        """读一个完整包，同步到 0xFA 0xFB 头。超时返回 None。"""
        ser = self.ser
        ser.timeout = timeout
        state = 0
        while True:
            b = ser.read(1)
            if not b:
                return None
            if state == 0:
                if b[0] == 0xFA:
                    state = 1
            else:
                if b[0] == 0xFB:
                    break
                if b[0] != 0xFA:
                    state = 0
        cnt_b = ser.read(1)
        if not cnt_b:
            return None
        cnt = cnt_b[0]
        data = ser.read(cnt)
        if len(data) < cnt:
            return None
        body = data[:-2]
        checksum_recv = struct.unpack(">H", data[-2:])[0]
        return {
            "type": data[0],
            "payload": data[1:-2],
            "checksum_ok": checksum_recv == calc_checksum(body),
            "raw": data,
        }

    # ── 命令下发 ─────────────────────────────────────────────────────
    def enable_motors(self, enable: bool = True) -> None:
        self.ser.write(build_command(ENABLE, 1 if enable else 0))

    def pulse(self) -> None:
        """喂看门狗（连接后须在约 2 秒内至少发一次命令）。"""
        self.ser.write(build_command(PULSE))

    def set_velocity(self, v_mms: int, w_degs: int) -> None:
        """设平移速度(mm/s)和旋转速度(deg/s)。首次调用前会先下发速度/加速度上限。"""
        if not self._limits_sent:
            self.ser.write(build_command(SETA, self.accel_mms2))
            self.ser.write(build_command(SETRA, self.accel_degs2))
            self.ser.write(build_command(SETV, self.max_v_mms))
            self.ser.write(build_command(SETRV, self.max_w_degs))
            self._limits_sent = True
        self.ser.write(build_command(VEL, v_mms))
        self.ser.write(build_command(RVEL, w_degs))

    def stop(self) -> None:
        self.ser.write(build_command(STOP))

    def close(self) -> None:
        """停止并断开连接（自动禁用电机）。"""
        if self.ser is None:
            return
        try:
            self.ser.write(build_command(STOP))
            self.ser.write(build_command(CLOSE))
            time.sleep(0.1)
        except Exception:
            pass
        finally:
            self.ser.close()
            self.ser = None
            self._connected = False

    # ── 数据读取 ─────────────────────────────────────────────────────
    def read_sip(self) -> dict | None:
        """读一个标准 SIP 并返回里程计字典；读不到或校验错返回 None。"""
        pkt = self._read_packet(timeout=0.5)
        if pkt is None or not pkt["checksum_ok"]:
            return None
        return parse_standard_sip(pkt)

    @property
    def connected(self) -> bool:
        return self._connected
