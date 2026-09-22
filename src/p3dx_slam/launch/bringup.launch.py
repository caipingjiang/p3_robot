import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    p3dx_slam_dir = get_package_share_directory('p3dx_slam')
    p3dx_desc_dir = get_package_share_directory('p3dx_description_ros')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(p3dx_slam_dir, 'worlds', 'p3dx_world.world')
    xacro_path = os.path.join(p3dx_desc_dir, 'urdf', 'pioneer3dx.xacro')

    # 0. 让 Gazebo 解析 package:// mesh（Gazebo classic 不认 package://）
    p3dx_desc_share = os.path.dirname(p3dx_desc_dir)  # .../install/p3dx_description_ros/share
    _existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = (
        p3dx_desc_share + os.pathsep + _existing_model_path
        if _existing_model_path else p3dx_desc_share
    )
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)

    # 1. Gazebo（加载 world）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    # 2. robot_description（xacro 生成）
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str
    )

    # 3. spawn 机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'p3dx'],
        output='screen',
    )

    # 4. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 5. joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
    ])
