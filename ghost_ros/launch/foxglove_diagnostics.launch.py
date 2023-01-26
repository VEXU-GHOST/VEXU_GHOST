import os

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    ghost_ros_base_dir = os.path.join(home_dir, "VEXU_GHOST", "ghost_ros")

    foxglove_diagnostics_node = Node(
        package='ghost_ros',
        executable='foxglove_diagnostics_node',
        name='foxglove_diagnostics_node',
        parameters=[ghost_ros_base_dir + "/config/foxglove_diagnostics_config.yaml"]
    )

    return LaunchDescription([
        foxglove_diagnostics_node
    ])