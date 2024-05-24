import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    home_dir = os.path.expanduser("~")
    pkg_share_dir = os.path.join(
        home_dir, "VEXU_GHOST", "03_ROS", "ghost_viz"
    )

    # This contains all the parameters for our ROS nodes
    pt_config_path = os.path.join(pkg_share_dir, "config/plotjuggler_config.xml")


    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        output="screen",
        arguments=["-n", "-l", pt_config_path]
    )

    return LaunchDescription([plotjuggler_node])
