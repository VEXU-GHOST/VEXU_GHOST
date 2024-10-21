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

    spawn_entity_args = (
        "-x 0.0 -y 0.0 -z 1.0 -R 0.0 -P 0.0 -Y 0.0 -entity ghost1 -topic robot_description"
    ).split()

    # Node to publish robot joint transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, {"robot_description": doc}],
        remappings=[("/sensors/wheel_odom", "/odom")],
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

    return [robot_state_publisher, joy_launch_description]


def generate_launch_description():
    # Load relevant filepaths
    ghost_sim_examples_dir = get_package_share_directory("ghost_sim_examples")
    # ghost_sim_share_dir = get_package_share_directory("ghost_sim")
    # rviz_config_path = os.path.join(ghost_sim_share_dir, "rviz/ekf_pf.rviz")

    
    # Simulator (Doesn't launch Simulator GUI by default, use CLI Arg "sim_gui" for debugging)
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ghost_sim_examples_dir,
                         'launch', 'rviz.launch.py')
        ),
    )

    # plot_juggler_node = Node(
    #     package="plotjuggler", executable="plotjuggler", name="plot_juggler"
    # )

    tank_mpc_viz_node = Node(
        package = "ghost_sim",
        executable = "tank_mpc_viz",
        name = "tank_mpc_viz_node" 
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_joy", default_value="false"),
            DeclareLaunchArgument(name="channel_id", default_value="1"),
            DeclareLaunchArgument("sim_gui", default_value="false"),
            DeclareLaunchArgument("verbose", default_value="true"),
            rviz2,
            # plot_juggler_node,
            tank_mpc_viz_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
