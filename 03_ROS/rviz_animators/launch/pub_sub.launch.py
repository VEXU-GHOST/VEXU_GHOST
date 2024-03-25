import launch
import launch_ros

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
        executable='subscriber_main',
    )

    return launch.LaunchDescription([
        publisher_node,
        subscriber_node
    ])