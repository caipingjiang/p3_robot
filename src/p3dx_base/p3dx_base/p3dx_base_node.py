# -*- coding: utf-8 -*-
"""Pioneer 3DX 底盘 ROS 2 驱动节点。

连接底盘串口（ARCOS 协议），解析标准 SIP，发布：
  - /odom   (nav_msgs/Odometry)   里程计
  - /tf     odom -> base_link     TF 变换
  - /battery(sensor_msgs/BatteryState)  电池电压
订阅：
  - /cmd_vel (geometry_msgs/Twist)  速度指令（内部转为 VEL/RVEL 下发）

运行：
  ros2 run p3dx_base p3dx_base_node --ros-args -p port:=/dev/p3dx
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

from p3dx_base.p3dx_driver import P3dxDriver

WATCHDOG_PERIOD = 2.0   # 秒，须 < 底盘看门狗默认 2s


class P3dxBaseNode(Node):
    def __init__(self):
        super().__init__("p3dx_base_node")

        # ── 参数 ──────────────────────────────────────────────────────
        self.declare_parameter("port", "/dev/p3dx")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("enable_motors", True)
        self.declare_parameter("max_v_mms", 500)      # 最大平移速度 mm/s
        self.declare_parameter("max_w_degs", 60)      # 最大旋转速度 deg/s
        self.declare_parameter("track_width_mm", 330.0)  # 轮距 mm（用于估算角速度）
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        enable_motors = self.get_parameter("enable_motors").value
        self.max_v_mms = self.get_parameter("max_v_mms").value
        self.max_w_degs = self.get_parameter("max_w_degs").value
        self.track_width_mm = self.get_parameter("track_width_mm").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # ── 发布者 / TF / 订阅 ────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        # ── 串口驱动（读线程与 cmd_vel 回调并发，用锁保护串口写）──────
        self._write_lock = threading.Lock()
        self.driver = P3dxDriver(
            port=port, baud=baud,
            max_v_mms=self.max_v_mms, max_w_degs=self.max_w_degs,
        )

        # ── 连接底盘 ──────────────────────────────────────────────────
        self.get_logger().info(f"连接底盘 {port} @ {baud} ...")
        try:
            self.driver.connect()
        except ConnectionError as e:
            self.get_logger().error(str(e))
            raise
        self.get_logger().info(
            f"已连接底盘：name={self.driver.name!r} "
            f"type={self.driver.robot_type!r} subtype={self.driver.subtype!r}")
        if enable_motors:
            with self._write_lock:
                self.driver.enable_motors(True)
            self.get_logger().info("电机已使能")

        # ── 后台读 SIP 线程 ───────────────────────────────────────────
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    # ── /cmd_vel 回调 ────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist) -> None:
        v_mms = int(round(msg.linear.x * 1000.0))                 # m/s -> mm/s
        w_degs = int(round(msg.angular.z * 180.0 / math.pi))      # rad/s -> deg/s
        v_mms = max(-self.max_v_mms, min(self.max_v_mms, v_mms))
        w_degs = max(-self.max_w_degs, min(self.max_w_degs, w_degs))
        with self._write_lock:
            self.driver.set_velocity(v_mms, w_degs)

    # ── 后台读循环 ───────────────────────────────────────────────────
    def _read_loop(self) -> None:
        last_pulse = time.time()
        while self._running and rclpy.ok():
            sip = self.driver.read_sip()
            if sip is None:
                continue
            if not self._running:
                break
            try:
                self._publish_sip(sip)
            except Exception:
                break  # 节点关闭时 publish 竞态，安全退出
            if time.time() - last_pulse >= WATCHDOG_PERIOD:
                try:
                    with self._write_lock:
                        self.driver.pulse()
                except Exception:
                    break
                last_pulse = time.time()

    # ── 发布里程计 / TF / 电池 ───────────────────────────────────────
    def _publish_sip(self, sip: dict) -> None:
        now = self.get_clock().now().to_msg()

        x = sip["x_mm"] / 1000.0
        y = sip["y_mm"] / 1000.0
        th = sip["th_rad"]
        # 线速度/角速度：由左右轮速估算（角速度需轮距，只影响 twist 精度，不影响 pose）
        v = (sip["lvel_mms"] + sip["rvel_mms"]) / 2.0 / 1000.0
        w = (sip["rvel_mms"] - sip["lvel_mms"]) / self.track_width_mm

        # 绕 z 轴旋转四元数
        qz = math.sin(th / 2.0)
        qw = math.cos(th / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # 协方差：给一个经验值（避免全 0 让 amcl 认为里程计完美）
        cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
        odom.pose.covariance = cov
        odom.twist.covariance = cov
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        battery = BatteryState()
        battery.header.stamp = now
        battery.voltage = float(sip["battery_v"])
        battery.percentage = float("nan")
        self.battery_pub.publish(battery)

    # ── 关闭 ─────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        with self._write_lock:
            self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = P3dxBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass  # SIGINT 时 rclpy 已自动 shutdown，这里重复调用会报错，忽略即可


if __name__ == "__main__":
    main()
