import os

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    swerve_share_dir = get_package_share_directory("ghost_ros")

    rplidar_node = Node(
        package='rplidar_ros2',
        executable='rplidar_scan_publisher',
        name='rplidar_scan_publisher',
        parameters=[{"frame_id": "lidar_link"}]
    )

    ghost_ros_main_node = Node(
        package='ghost_ros',
        executable='ghost_ros_main',
        name='ghost_ros_main',
        output='screen',
    )

    return LaunchDescription([
        ghost_ros_main_node,
        rplidar_node,
    ])