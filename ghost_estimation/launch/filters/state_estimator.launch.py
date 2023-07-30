from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    state_estimator_dir = os.path.join(get_package_share_directory("ghost_estimation"))
    
    rle_ekf_launch_= IncludeLaunchDescription(PythonLaunchDescriptionSource(
       os.path.join(get_package_share_directory("robot_localization")) + '/launch/ekf.launch.py'
    ))

    state_estimator_node_ = Node(
            package = 'ghost_estimation',
            executable = 'state_estimator_node',
            name = 'state_estimator_node',
            output = 'screen',
            parameters = [state_estimator_dir, 'config', 'state_estimator.yaml']
        )

    return LaunchDescription([
        state_estimator_node_,
        rle_ekf_launch_
   ])