import os
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# Opaque Function hack to allow for better CLI arg parsing
def launch_setup(context, *args, **kwargs):
    ghost_sim_share_dir = get_package_share_directory("ghost_sim")
    filename = "test_tank_init.xacro"

    # Load XACRO and process to urdf then to text
    xacro_path = os.path.join(ghost_sim_share_dir, "urdf", filename)
    xml = xacro.process_file(xacro_path)
    doc = xml.toprettyxml(indent="  ")

    # Node to publish robot joint transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, {"robot_description": doc}],
    )

    # Joystick (Only launched if joystick CLI arg is set to True)
    joy_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ghost_sim_share_dir, "launch", "joystick.launch.py")
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_joy")),
        launch_arguments={
            "channel_id": LaunchConfiguration("channel_id"),
        }.items(),
    )

    return [joy_launch_description]


def generate_launch_description():
    # Load relevant filepaths
    ghost_ros_share_dir = get_package_share_directory("ghost_ros_interfaces")
    ghost_over_under_share_dir = get_package_share_directory("ghost_over_under")

    ghost_sim_share_dir = get_package_share_directory("ghost_sim")
    ghost_localization_share_dir = get_package_share_directory("ghost_localization")

    home_dir = os.path.expanduser("~")
    ghost_ros_base_dir = os.path.join(
        home_dir, "VEXU_GHOST", "03_ROS", "ghost_ros_interfaces"
    )

    rviz_config_path = os.path.join(ghost_sim_share_dir, "rviz/bag_playback.rviz")

    ekf_pf_node = Node(
        package="ghost_localization",
        executable="ekf_pf_node",
        name="ekf_pf_node",
        output="screen",
        parameters=[ghost_over_under_share_dir + "/config/ros_config.yaml"],
    )

    # Launch RVIZ Display as primary GUI interface
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization_node",
        output="screen",
        parameters=[ghost_over_under_share_dir + "/config/ros_config.yaml"],
    )

    bag_set_pose_time = Node(
        package="ghost_sim",
        executable="bag_set_pose_time",
        name="bag_set_pose_time",
        output="screen",
    )

    plot_juggler_node = Node(
        package="plotjuggler", executable="plotjuggler", name="plot_juggler"
    )

    #  ros2 bag play bag2 --clock --remap '/odometry/filtered:=/odometry/filtered_null' '/tf:=/tf_null'
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_joy", default_value="false"),
            DeclareLaunchArgument(name="channel_id", default_value="1"),
            DeclareLaunchArgument("sim_gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="true"),
            ekf_pf_node,
            rviz_node,
            plot_juggler_node,
            robot_localization_node,
            bag_set_pose_time,
            OpaqueFunction(function=launch_setup),
        ]
    )
