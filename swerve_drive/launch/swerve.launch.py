import os

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():
    swerve_share_dir = get_package_share_directory("swerve_drive")

    joy_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                swerve_share_dir,
                "launch",
                "joystick.launch.py"
            )
        )
    )

    rviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                swerve_share_dir,
                "launch",
                "rviz.launch.py"
            )
        )
    )

    return LaunchDescription([
        joy_launch_description,
        rviz_launch_description,
        # simulator
    ])