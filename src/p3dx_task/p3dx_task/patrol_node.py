#!/usr/bin/env python3
"""P3DX 固定巡逻节点：按顺序循环导航到预设点。"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from p3dx_task.waypoints import load_waypoints, load_loop, yaw_to_quaternion
from p3dx_task.patrol_core import PatrolStatus, next_index


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")
        self.declare_parameter("waypoints_file", "")

        wp_file = self.get_parameter("waypoints_file").value
        if not wp_file:
            self.get_logger().error("未指定 waypoints_file 参数")
            raise RuntimeError("waypoints_file 为空")

        try:
            self._waypoints = load_waypoints(wp_file)
            self._loop = load_loop(wp_file)
        except (OSError, ValueError) as e:
            self.get_logger().error(f"加载巡逻点失败：{e}")
            raise RuntimeError(f"加载巡逻点失败: {e}") from e
        if not self._waypoints:
            self.get_logger().error("巡逻点列表为空")
            raise RuntimeError("waypoints 为空")
        self._current_index = 0  # 当前正在导航的巡逻点索引
        self._status = PatrolStatus.IDLE
        self._goal_handle = None
        self._retry_count = 0
        self._max_retries = 15  # goal 被拒最多重试 15 次（约 30s），应对 nav2 未 activate
        self._retry_timer = None  # 单一重试 timer（避免 create_timer 默认 repeating 泄漏）

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info(
            f"加载 {len(self._waypoints)} 个巡逻点，loop={self._loop}"
        )

        self.get_logger().info("等待 navigate_to_pose action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server 不可用")
            raise RuntimeError("action server 超时")

        self._navigate_to(self._waypoints[0])  # 启动：导航到第一个点

    def _advance(self):
        """到达当前点后，推进到下一个巡逻点。"""
        nxt = next_index(self._current_index, len(self._waypoints), self._loop)
        if nxt is None:
            self.get_logger().info("巡逻完成（非循环），进入 IDLE")
            self._status = PatrolStatus.IDLE
            return
        self._current_index = nxt
        self._navigate_to(self._waypoints[nxt])

    def _navigate_to(self, wp):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        q = yaw_to_quaternion(wp.yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self._status = PatrolStatus.NAVIGATING
        self.get_logger().info(
            f"导航到点 [{wp.name}] ({wp.x:.2f}, {wp.y:.2f}, yaw={wp.yaw:.2f})"
        )
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._retry_count += 1
            if self._retry_count > self._max_retries:
                self.get_logger().error(
                    f"目标连续被拒 {self._max_retries} 次，放弃巡逻"
                )
                self._status = PatrolStatus.FAILED
                return
            self.get_logger().warn(
                f"目标被 nav2 拒绝（{self._retry_count}/{self._max_retries}），2 秒后重试"
            )
            if self._retry_timer is None:
                self._retry_timer = self.create_timer(2.0, self._retry_navigate)
            return
        self._retry_count = 0
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _retry_navigate(self):
        """重试导航到当前巡逻点（one-shot：取消自己的 timer）。"""
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._navigate_to(self._waypoints[self._current_index])

    def _result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"到达点 [{self._waypoints[self._current_index].name}]"
            )
            self._status = PatrolStatus.ARRIVED
            self._advance()
        else:
            self.get_logger().warn(f"导航失败，status={result.status}")
            self._status = PatrolStatus.FAILED


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
