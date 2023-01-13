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

    serial_node = Node(
        package='ghost_ros',
        executable='jetson_v5_serial_node',
        name='ghost_serial_node',
        output='screen',
        parameters=[os.path.join(swerve_share_dir, "config", "ghost_serial_config.yaml")]
    )

    estimator_node = Node(
        package='ghost_ros',
        executable='ghost_estimator_node',
        name='ghost_estimator_node',
        output='screen',
        parameters=[os.path.join(swerve_share_dir, "config", "ghost_estimator_config.yaml")]

    )

    state_machine_node = Node(
        package='ghost_ros',
        executable='robot_state_machine_node',
        name='ghost_state_machine_node',
        output='screen',
        parameters=[os.path.join(swerve_share_dir, "config", "ghost_state_machine_config.yaml")]
    )

    return LaunchDescription([
        serial_node,
        estimator_node,
        state_machine_node,
        rplidar_node,
    ])