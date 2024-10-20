import os

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    home_dir = os.path.expanduser("~")
    ghost_ros_base_dir = os.path.join(home_dir, "VEXU_GHOST", "ghost_ros")

    rplidar_node = Node(
        package="rplidar_ros2",
        executable="rplidar_scan_publisher",
        name="rplidar_scan_publisher",
        parameters=[{"frame_id": "lidar_link"}],
    )

    serial_node = Node(
        package="ghost_ros",
        executable="jetson_v5_serial_node",
        name="ghost_serial_node",
        output="screen",
        parameters=[ghost_ros_base_dir + "/config/ghost_serial_config.yaml"],
    )

    estimator_node = Node(
        package="ghost_ros",
        executable="ghost_estimator_node",
        name="ghost_estimator_node",
        output="screen",
        parameters=[ghost_ros_base_dir + "/config/ghost_estimator_config.yaml"],
    )

    state_machine_node = Node(
        package="ghost_ros",
        executable="robot_state_machine_node",
        name="ghost_state_machine_node",
        output="screen",
        parameters=[ghost_ros_base_dir + "/config/ghost_state_machine_config.yaml"],
    )

    foxglove_diagnostics_node = Node(
        package="ghost_ros",
        executable="foxglove_diagnostics_node",
        name="foxglove_diagnostics_node",
        parameters=[ghost_ros_base_dir + "/config/foxglove_diagnostics_config.yaml"],
    )

    return LaunchDescription(
        [
            # foxglove_diagnostics_node,
            # serial_node,
            estimator_node,
            state_machine_node,
            # rplidar_node,
        ]
    )
