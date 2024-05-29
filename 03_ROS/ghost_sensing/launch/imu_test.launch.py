import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sensing_share = launch_ros.substitutions.FindPackageShare(
        package="ghost_sensing"
    ).find("ghost_sensing")
    imu_filter_config_path = os.path.join(sensing_share, "config/imu_filter_node.yaml")
    rviz_config_path = os.path.join(sensing_share, "rviz/imu.rviz")

    imu_filter_node = launch_ros.actions.Node(
        package="ghost_sensing",
        executable="imu_filter_node",
        name="imu_filter_node",
        output="screen",
        parameters=[imu_filter_config_path],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    realsense_share = launch_ros.substitutions.FindPackageShare(
        package="realsense2_camera"
    ).find("realsense2_camera")
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "unite_imu_method": "2",
            "enable_depth": "false",
            "enable_color": "false",
            "enable_sync": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "gyro_fps": "200",  # 200 or 400
            "accel_fps": "63",  # 63 or 250
        }.items(),
    )

    return launch.LaunchDescription([realsense, imu_filter_node, rviz_node])
