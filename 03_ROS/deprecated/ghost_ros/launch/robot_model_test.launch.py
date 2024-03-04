import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='ghost_description').find('ghost_description')
    ros_pkg_share = launch_ros.substitutions.FindPackageShare(package='ghost_ros').find('ghost_ros')
    default_model_path = os.path.join(description_pkg_share, 'urdf/ghost1.urdf')
    default_rviz_config_path = os.path.join(ros_pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        name = "robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
