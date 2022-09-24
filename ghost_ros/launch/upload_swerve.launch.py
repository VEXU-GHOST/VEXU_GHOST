import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    ghost_ros_share_dir = get_package_share_directory('ghost_ros')
    
    urdf_path = os.path.join(ghost_ros_share_dir, "urdf", "swerve.urdf")

    doc = xacro.process(urdf_path)
    
    spawn_entity_args = ("-x 0.0 -y 0.0 -z 1.0 -R 0.0 -P 0.0 -Y 0.0 -entity swerve -topic robot_description").split()
    
    with open(os.path.join(ghost_ros_share_dir, "urdf", "robot_description.urdf"), 'w') as file:
        file.write(doc)

    return LaunchDescription([
        Node(
            package = "gazebo_ros",
            executable = "spawn_entity.py",
            output='screen',
            arguments=spawn_entity_args
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}, {"robot_description": doc}],
        )
    ])