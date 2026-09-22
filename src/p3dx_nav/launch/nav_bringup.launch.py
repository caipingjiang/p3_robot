import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    p3dx_nav_dir = get_package_share_directory('p3dx_nav')
    p3dx_slam_dir = get_package_share_directory('p3dx_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = os.path.join(p3dx_nav_dir, 'maps', 'p3dx_map.yaml')
    params_file = os.path.join(p3dx_nav_dir, 'config', 'nav2_params.yaml')
    rviz_path = os.path.join(p3dx_nav_dir, 'config', 'nav_rviz.rviz')

    # 1. 共享 bringup（gazebo + 机器人）
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(p3dx_slam_dir, 'launch', 'bringup.launch.py')
        ),
    )

    # 2. nav2（slam=false → map_server + amcl + planner/controller/bt + lifecycle）
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'params_file': params_file,
            'autostart': 'true',
            'use_sim_time': 'true',
        }.items(),
    )

    # 3. rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
    )

    return LaunchDescription([
        bringup,
        nav2,
        rviz,
    ])
