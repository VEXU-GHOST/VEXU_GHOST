import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ghost_high_stakes').find('ghost_high_stakes')
    xacro_path = os.path.join(pkg_share, 'urdf/tankbot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        name = "robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_path])}]
    )
    
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        joint_state_publisher_gui_node,
       
        robot_state_publisher_node,
        rviz_node
    ])