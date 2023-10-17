import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    ghost_ros_base_dir = os.path.join(home_dir, "VEXU_GHOST", "03_ROS", "ghost_ros")
    ros_pkg_share = FindPackageShare(package='ghost_ros').find('ghost_ros')
    lidar_rviz_config_path = os.path.join(ros_pkg_share, 'rviz/lidar_scan.rviz')
    particle_filter_rviz_config_path = os.path.join(ros_pkg_share, 'rviz/particle_filter.rviz')

    estimator_node = Node(
        package='ghost_ros',
        executable='ghost_estimator_node',
        name='ghost_estimator_node',
        output='screen',
        parameters=[ghost_ros_base_dir + "/config/ghost_estimator_config.yaml"]
    )

    lidar_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lidar_rviz',
        output='screen',
        arguments=['-d', lidar_rviz_config_path],
    )

    particle_filter_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='particle_filter_rviz',
        output='screen',
        arguments=['-d', particle_filter_rviz_config_path],
    )

    return LaunchDescription([
        estimator_node,
        lidar_rviz_node,
        particle_filter_rviz_node,
    ])