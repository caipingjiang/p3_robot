from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="p3dx_base",
            executable="p3dx_base_node",
            name="p3dx_base",
            output="screen",
            parameters=[{
                "port": "/dev/p3dx",
                "baud": 9600,
                "enable_motors": True,
                "max_v_mms": 500,
                "max_w_degs": 60,
                "track_width_mm": 330.0,
                "odom_frame": "odom",
                "base_frame": "base_link",
            }],
        )
    ])
