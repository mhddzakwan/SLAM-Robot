from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro', ' ', urdf_path])


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='robot_base',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[{'serial_port': '/dev/ttyACM0'}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['--x', '0.06', '--y', '0', '--z', '0.10',
                '--yaw', '4.71239', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_link'],
        ),
            Node(
            package='robot_base',
            executable='lidar_node',
            name='lidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )
            ),
            launch_arguments={
                'slam_params_file': os.path.join(pkg_share, 'config', 'slam_toolbox.yaml'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])