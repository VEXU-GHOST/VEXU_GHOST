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
    ghost_viz_share_dir = get_package_share_directory("ghost_visualization_assets")
    filename = "baby_jerry_sim.urdf.xacro" # Updated to your new RealSense wrapper!

    # 1. Load Baby Jerry Wrapper XACRO
    xacro_path = os.path.join(ghost_sim_share_dir, "urdf", filename)
    xml = xacro.process_file(xacro_path)
    doc = xml.toprettyxml(indent="  ")

    spawn_robot_args = (
        "-x 0.0 -y 0.0 -z 0.1 -R 0.0 -P 0.0 -Y 0.0 -entity baby_jerry -topic robot_description"
    ).split()

    # Node to spawn Baby Jerry in Gazebo
    gazebo_ros = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=spawn_robot_args,
        parameters=[{"use_sim_time": True}],
    )

    # 2. Load and spawn the competition field assembly URDF
    field_urdf_path = os.path.join(
        ghost_viz_share_dir, "field_assembly", "urdf", "field_assembly.urdf"
    )
    spawn_field_args = (
        f"-file {field_urdf_path} -entity field_assembly -x 0.0 -y 0.0 -z 0.0"
    ).split()

    spawn_field = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=spawn_field_args,
        parameters=[{"use_sim_time": True}],
    )

    # Node to publish robot joint transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, {"robot_description": doc}],
        remappings=[("/sensors/wheel_odom", "/odom")],
    )

    # Joystick
    joy_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ghost_sim_share_dir, "launch", "joystick.launch.py")
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_joy")),
        launch_arguments={
            "channel_id": LaunchConfiguration("channel_id"),
        }.items(),
    )

    return [spawn_field, gazebo_ros, robot_state_publisher, joy_launch_description]


def generate_launch_description():
    # Load relevant filepaths
    gazebo_ros_share_dir = get_package_share_directory("gazebo_ros")
    ghost_ros_share_dir = get_package_share_directory("ghost_ros_interfaces")
    ghost_sim_share_dir = get_package_share_directory("ghost_sim")
    ghost_localization_share_dir = get_package_share_directory("ghost_localization")
    ghost_high_stakes_share_dir = get_package_share_directory("ghost_high_stakes")

    home_dir = os.path.expanduser("~")
    ghost_ros_base_dir = os.path.join(
        home_dir, "VEXU_GHOST", "03_ROS", "ghost_ros_interfaces"
    )

    world_file = os.path.join(ghost_sim_share_dir, "worlds", "spin_up.world")
    rviz_config_path = os.path.join(ghost_high_stakes_share_dir, "rviz/world_config.rviz")

    # Simulator (Doesn't launch Simulator GUI by default, use CLI Arg "sim_gui" for debugging)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "gui": LaunchConfiguration("sim_gui"),
            "verbose": LaunchConfiguration("verbose"),
        }.items(),
    )

    ekf_pf_node = Node(
        package="ghost_localization",
        executable="ekf_pf_node",
        name="ekf_pf_node",
        output="screen",
        parameters=[ghost_localization_share_dir + "/config/ekf_pf_node.yaml"],
        remappings=[("/sensors/wheel_odom", "/odom")],
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
        parameters=[ghost_ros_base_dir + "/config/robot_localization_config.yaml"],
        remappings=[("/sensors/wheel_odom", "/odom"),]
                    # ("/odometry/filtered", "/map_ekf/odometry")],
    )

    plot_juggler_node = Node(
        package="plotjuggler", executable="plotjuggler", name="plot_juggler"
    )

    sensor_update_spoofer_node = Node(
        package="ghost_sim",
        executable="sensor_update_spoofer",
        name="sensor_update_spoofer",
        output="screen",
        parameters=[ghost_high_stakes_share_dir + "/config/ros_config.yaml",
            {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_joy", default_value="false"),
            DeclareLaunchArgument(name="channel_id", default_value="1"),
            DeclareLaunchArgument("sim_gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="true"),
            simulation,
            ekf_pf_node,
            rviz_node,
            # plot_juggler_node,
            robot_localization_node,
            sensor_update_spoofer_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
