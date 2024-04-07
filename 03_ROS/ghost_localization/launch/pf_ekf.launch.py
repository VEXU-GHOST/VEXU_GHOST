import os
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Does not launch physics simulator (Gazebo), but launches RVIZ
# Must be launched in tandem with a robot launch file (like the test diff drive in ghost_sim and, hopefully, the hardware)
# Opaque Function hack to allow for better CLI arg parsing

def generate_launch_description():
    # Load relevant filepaths
    ghost_localization_share_dir = get_package_share_directory('ghost_localization')
    rviz_config_path = os.path.join(ghost_localization_share_dir, 'rviz/ekf_pf.rviz')

    # Launch RVIZ Display as primary GUI interface
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    pf_ekf_node = Node(
        package='ghost_localization',
        executable='pf_ekf_node',
        name='pf_ekf_node',
        output='screen',
        parameters=[ghost_localization_share_dir + "/config/ekf_pf_node.yaml"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='channel_id', default_value='1'),
        DeclareLaunchArgument('verbose', default_value='true'),
        # rviz_node,
        pf_ekf_node
        # plot_juggler_node,
    ])