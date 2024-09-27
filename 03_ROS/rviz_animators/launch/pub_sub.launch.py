import launch
import launch_ros

import os

def generate_launch_description():

    home_dir = os.path.expanduser('~')
    rviz_animators_dir = os.path.join(home_dir, "VEXU_GHOST", "03_ROS", "rviz_animators")

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(rviz_animators_dir, "config/test.yaml")

    publisher_node = launch_ros.actions.Node(
        name = "publisher_node",
        package = 'rviz_animators',
        # This name is specified in the CmakeLists.txt file of this package 
        executable = 'publisher',
    )

    subscriber_node = launch_ros.actions.Node(
        name='subscriber_node',
        package= 'rviz_animators',
        # This name is specified in the CmakeLists.txt file of this package 
        executable='subscriber_main',
        parameters=[ros_config_file]
    )

    return launch.LaunchDescription([
        publisher_node,
        subscriber_node
    ])