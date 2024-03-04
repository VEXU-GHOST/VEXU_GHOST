import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ghost_over_under').find('ghost_over_under')
    xacro_path = os.path.join(pkg_share, 'urdf/ghost_15.xacro')
    # rviz_config_path = os.path.join(pkg_share, 'rviz/base_link_config.rviz')
    rviz_config_path = os.path.join(pkg_share, 'rviz/odom_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        name = "robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_path])}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
