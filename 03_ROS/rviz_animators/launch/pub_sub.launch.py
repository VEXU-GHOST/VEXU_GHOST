import launch
import launch_ros

import os

# <launch>
#     <!-- Other nodes and configurations -->

#     <!-- Launch RViz -->
#     <node pkg="rviz" type="rviz" name="rviz" args=" $(find my_package)/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/pub_sub.launch.py" />
# </launch>

def generate_launch_description():

    home_dir = os.path.expanduser('~')
    rviz_animators_directory = os.path.join(home_dir, "VEXU_GHOST", "03_ROS", "rviz_animators")
    yaml_path = os.path.join(rviz_animators_directory, "config", "test.yaml")

    publisher_node = launch_ros.actions.Node(
        name = "publisher_node",
        package = 'rviz_animators',
        # This name is specified in the CmakeLists.txt file of this package 
        executable = 'publisher',
        parameters=[yaml_path]
    )

    subscriber_node = launch_ros.actions.Node(
        package= 'rviz_animators',
        # This name is specified in the CmakeLists.txt file of this package 
        executable='main',
        output='screen',
        parameters=[yaml_path]
    )

    return launch.LaunchDescription([
        publisher_node,
        subscriber_node
    ])