import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    swerve_drive_share_dir = get_package_share_directory('swerve_drive')
    world_file = os.path.join(swerve_drive_share_dir, "urdf", "spin_up.world")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir,
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    upload_swerve = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swerve_drive_share_dir, 'launch',
                            'upload_swerve.launch.py')
        )
    )

    return LaunchDescription([
        gz,
        upload_swerve,
    ])
