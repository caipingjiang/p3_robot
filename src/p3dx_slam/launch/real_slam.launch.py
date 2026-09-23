# -*- coding: utf-8 -*-
"""真机 SLAM 建图 launch：底盘驱动 + 激光 + 静态 TF + slam_toolbox + rviz。

用法：
  ros2 launch p3dx_slam real_slam.launch.py
  ros2 launch p3dx_slam real_slam.launch.py base_port:=/dev/ttyUSB0   # udev 规则没装时
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p3dx_slam_dir = get_package_share_directory('p3dx_slam')

    # ── 可调参数声明 ─────────────────────────────────────────────────
    declare_base_port = DeclareLaunchArgument('base_port', default_value='/dev/p3dx',
                                              description='底盘串口（udev 装好后固定 /dev/p3dx）')
    declare_sensor_ip = DeclareLaunchArgument('sensor_ip', default_value='192.168.8.2',
                                              description='激光雷达 IP（USB 连接固定）')
    declare_laser_x = DeclareLaunchArgument('laser_x', default_value='0.16',
                                            description='激光相对 base_link 前方距离 m')
    declare_laser_y = DeclareLaunchArgument('laser_y', default_value='0.0',
                                            description='激光相对 base_link 左右偏移 m')
    declare_laser_z = DeclareLaunchArgument('laser_z', default_value='0.15',
                                            description='激光相对 base_link 高度 m')

    base_port = LaunchConfiguration('base_port')
    sensor_ip = LaunchConfiguration('sensor_ip')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')

    # ── 1. 底盘驱动（odom / TF odom->base_link）──────────────────────
    p3dx_base = Node(
        package='p3dx_base',
        executable='p3dx_base_node',
        name='p3dx_base',
        output='screen',
        parameters=[{'port': base_port, 'enable_motors': True}],
    )

    # ── 2. 激光驱动（/scan, frame_id=laser）─────────────────────────
    lakibeam = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node0',
        output='screen',
        parameters=[{
            'frame_id': 'laser',
            'output_topic': 'scan',
            'inverted': False,
            'hostip': '0.0.0.0',
            'port': '2368',
            'angle_offset': 0,
            'sensorip': sensor_ip,
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315',
        }],
    )

    # ── 3. 静态 TF base_link -> laser（安装位置，暂定仿真值，待实测）──
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_laser_tf',
        arguments=[laser_x, laser_y, laser_z, '0', '0', '0', 'base_link', 'laser'],
    )

    # ── 4. slam_toolbox（online sync，真机时间）──────────────────────
    slam_params_path = os.path.join(p3dx_slam_dir, 'config', 'slam_toolbox_params.yaml')
    with open(slam_params_path, 'r') as f:
        slam_params = yaml.safe_load(f)['slam_toolbox']['ros__parameters']
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
    )

    # ── 5. rviz ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(p3dx_slam_dir, 'config', 'rviz_slam.rviz')],
        output='screen',
    )

    return LaunchDescription([
        declare_base_port, declare_sensor_ip,
        declare_laser_x, declare_laser_y, declare_laser_z,
        p3dx_base, lakibeam, static_tf, slam_toolbox, rviz,
    ])
