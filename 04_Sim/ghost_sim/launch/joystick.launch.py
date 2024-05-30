from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    channel_id = context.perform_substitution(LaunchConfiguration("channel_id"))
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joystick_node",
        parameters=[{"dev": "/dev/input/js" + channel_id}],
    )

    return [joy_linux_node]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="channel_id", default_value="1"),
            OpaqueFunction(function=launch_setup),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[
                    {
                        "require_enable_button": False,
                        "axis_linear": {"x": 0, "y": 1},
                        "axis_angular": {"yaw": 3},
                        "scale_linear": {"x": -1.0, "y": 1.0},
                        "scale_angular": {"yaw": 0.5},
                    }
                ],
            ),
        ]
    )
