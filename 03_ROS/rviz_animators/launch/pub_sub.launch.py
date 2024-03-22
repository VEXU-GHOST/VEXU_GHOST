import launch
import launch_ros


# <launch>
#     <!-- Other nodes and configurations -->

#     <!-- Launch RViz -->
#     <node pkg="rviz" type="rviz" name="rviz" args=" $(find my_package)/home/annievu/VEXU_GHOST/03_ROS/rviz_animators/pub_sub.launch.py" />
# </launch>

def generate_launch_description():

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
        executable='main',
    )

    return launch.LaunchDescription([
        publisher_node,
        subscriber_node
    ])


    "$(find my_package)/path/to/your_rviz_config_file.rviz"