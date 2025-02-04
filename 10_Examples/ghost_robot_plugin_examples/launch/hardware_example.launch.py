import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    home_dir = os.path.expanduser("~")
    pkg_dir = os.path.join(
        home_dir, "VEXU_GHOST", "10_Examples", "ghost_robot_plugin_examples"
    )

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(pkg_dir, "config/example_ros_config.yaml")

    # This contains all the port and device info that gets compiled on to the V5 Brain
    robot_config_yaml_path = os.path.join(
        pkg_dir, "config/example_hardware_config.yaml"
    )

    plugin_type = "ghost_robot_plugin_examples::GhostRobotPluginExample"
    robot_name = "EXAMPLE_ROBOT"

    ########################
    ### Node Definitions ###
    ########################
    serial_node = Node(
        package="ghost_ros_interfaces",
        executable="jetson_v5_serial_node",
        name="ghost_serial_node",
        output="screen",
        parameters=[
            ros_config_file,
            {"robot_config_yaml_path": robot_config_yaml_path},
        ],
    )

    competition_state_machine_node = Node(
        package="ghost_ros_interfaces",
        executable="competition_state_machine_node",
        output="screen",
        parameters=[
            ros_config_file,
            {
                "robot_config_yaml_path": robot_config_yaml_path,
            },
        ],
        arguments=[plugin_type, robot_name],
    )

    return LaunchDescription(
        [
            serial_node,
            competition_state_machine_node,
        ]
    )
