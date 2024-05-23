import launch
import launch_ros


def generate_launch_description():

    publisher_node = launch_ros.actions.Node(
        name="publisher_node",
        package="ghost_examples",
        # This name is specified in the CmakeLists.txt file of this package
        executable="ros_publisher_example",
    )

    subscriber_node = launch_ros.actions.Node(
        name="subscriber_node",
        package="ghost_examples",
        # This name is specified in the CmakeLists.txt file of this package
        executable="ros_subscriber_example_main",
    )

    return launch.LaunchDescription([publisher_node, subscriber_node])
