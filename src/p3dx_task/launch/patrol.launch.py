import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p3dx_task_dir = get_package_share_directory("p3dx_task")
    p3dx_slam_dir = get_package_share_directory("p3dx_slam")
    p3dx_nav_dir = get_package_share_directory("p3dx_nav")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(p3dx_slam_dir, "worlds", "p3dx_world.world"),
        description="仿真 world 文件路径（需与 map 配套）",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(p3dx_nav_dir, "maps", "p3dx_map.yaml"),
        description="nav2 导航地图 yaml（需与 world 配套）",
    )
    declare_waypoints = DeclareLaunchArgument(
        "waypoints",
        default_value=os.path.join(p3dx_task_dir, "config", "patrol_waypoints.yaml"),
        description="巡逻点列表 yaml",
    )

    world_path = LaunchConfiguration("world")
    map_path = LaunchConfiguration("map")
    waypoints_path = LaunchConfiguration("waypoints")

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(p3dx_nav_dir, "launch", "nav_bringup.launch.py")
        ),
        launch_arguments={"world": world_path, "map": map_path}.items(),
    )

    patrol = Node(
        package="p3dx_task",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[{"waypoints_file": waypoints_path}],
    )

    return LaunchDescription([declare_world, declare_map, declare_waypoints, nav, patrol])
