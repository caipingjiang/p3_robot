# -*- coding: utf-8 -*-
"""无线手柄遥控 launch：joy_node（读手柄 -> /joy）+ teleop_twist_joy（/joy -> /cmd_vel）。

用法：
  ros2 launch p3dx_slam joy_teleop.launch.py               # 默认手柄 SDL 索引 0
  ros2 launch p3dx_slam joy_teleop.launch.py joy_dev:=1    # 被键盘等抢号时，改设备索引
  ros2 launch p3dx_slam joy_teleop.launch.py joy_name:=XXX # 或按设备名匹配（优先于 joy_dev）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p3dx_slam_dir = get_package_share_directory('p3dx_slam')
    joy_config = os.path.join(p3dx_slam_dir, 'config', 'joy_xbox.yaml')

    # 通用默认：按 SDL 索引选手柄；需要时可用 joy_name 覆盖成按名匹配
    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev', default_value='0',
        description='手柄 SDL 设备索引（默认 0）')
    declare_joy_name = DeclareLaunchArgument(
        'joy_name', default_value='',
        description='可选：手柄设备名（非空时按名匹配，优先于 joy_dev）')

    # 1. 读手柄 -> /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_config, {
            'device_id': LaunchConfiguration('joy_dev'),
            'device_name': LaunchConfiguration('joy_name'),
        }],
    )

    # 2. /joy -> /cmd_vel（映射参数在 joy_xbox.yaml 的 teleop_twist_joy_node 段）
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
    )

    return LaunchDescription([
        declare_joy_dev,
        declare_joy_name,
        joy_node,
        teleop_node,
    ])
