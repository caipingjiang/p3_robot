import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    p3dx_slam_dir = get_package_share_directory('p3dx_slam')

    rviz_path = os.path.join(p3dx_slam_dir, 'config', 'rviz_slam.rviz')
    slam_params_path = os.path.join(p3dx_slam_dir, 'config', 'slam_toolbox_params.yaml')

    # 1. 共享 bringup（gazebo + 机器人）
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(p3dx_slam_dir, 'launch', 'bringup.launch.py')
        ),
    )

    # 2. slam_toolbox（online sync）
    with open(slam_params_path, 'r') as f:
        slam_params = yaml.safe_load(f)['slam_toolbox']['ros__parameters']
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
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
        slam_toolbox,
        rviz,
    ])
