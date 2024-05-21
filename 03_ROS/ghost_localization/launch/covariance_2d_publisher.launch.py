import os
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Load relevant filepaths
    ghost_localization_share_dir = get_package_share_directory('ghost_localization')

    covariance_2d_publisher_node = Node(
        package='ghost_localization',
        executable='covariance_2d_publisher',
        output='screen',
        parameters=[ghost_localization_share_dir + "/config/covariance_2d_publisher_test.yaml"]
    )

    return LaunchDescription([
        covariance_2d_publisher_node
    ])