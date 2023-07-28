from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    state_estimator_node_ = 
        Node(
            package = 'ghost_estimation',
            executable = 
            name 'state_estimator'
        )


    return LaunchDescription([
        state_estimator_node_
   ])