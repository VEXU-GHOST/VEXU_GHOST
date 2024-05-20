import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    ghost_over_under_base_dir = os.path.join(home_dir, "VEXU_GHOST", "11_Robots", "ghost_over_under")
    ros_config_file = os.path.join(ghost_over_under_base_dir, "config/ros_config.yaml")

    pkg_share = launch_ros.substitutions.FindPackageShare(package='ghost_over_under').find('ghost_over_under')
    xacro_path = os.path.join(pkg_share, 'urdf/ghost_15.xacro')
    # rviz_config_path = os.path.join(pkg_share, 'rviz/base_link_config.rviz')
    rviz_config_path = os.path.join(pkg_share, 'rviz/odom_config.rviz')
    rviz_config_path = os.path.join(pkg_share, 'rviz/world_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        name = "robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_path]), 'ignore_timestamp': True}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    covariance_2d_publisher_node = launch_ros.actions.Node(
        package='ghost_localization',
        executable='covariance_2d_publisher',
        output='screen',
        parameters=[ros_config_file]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        covariance_2d_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
