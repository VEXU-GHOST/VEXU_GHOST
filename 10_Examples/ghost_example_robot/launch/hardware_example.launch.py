import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory("ghost_example_robot")

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(pkg_dir, "config/example_ros_config.yaml")

    # This contains all the port and device info that gets compiled on to the V5 Brain
    robot_config_yaml_path = os.path.join(
        pkg_dir, "config/example_hardware_config.yaml"
    )

    plugin_type = "ghost_example_robot::GhostExampleRobot"
    robot_name = "EXAMPLE_ROBOT"

    ########################
    ### Node Definitions ###
    ########################

    urdf_path = os.path.join(pkg_dir, "urdf", "example_robot.urdf.xacro")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": xacro.process_file(urdf_path).toxml()}],
    )

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

    rplidar_node = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[
                {
                    "channel_type": "serial",
                    "serial_port": "/dev/ttyUSB0",
                    "serial_baudrate": 256000,
                    "frame_id": "lidar_link",
                    "inverted": False,
                    "angle_compensate": True,
                }
            ],
    )

    return LaunchDescription(
        [
            serial_node,
            competition_state_machine_node,
            rplidar_node,
            robot_state_publisher
        ]
    )
