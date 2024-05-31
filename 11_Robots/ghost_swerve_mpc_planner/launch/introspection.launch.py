import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    home_dir = os.path.expanduser("~")
    config_file = os.path.join(
        home_dir,
        "VEXU_GHOST",
        "11_Robots",
        "ghost_swerve_mpc_planner",
        "config",
        "swerve_mpc_config.yaml",
    )

    #########################
    #### Launch Includes ####
    #########################
    plotjuggler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ghost_viz"),
                "launch",
                "trajectory_plotter.launch.py",
            )
        )
    )

    ########################
    ### Node Definitions ###
    ########################
    swerve_animator_node = Node(
        package="ghost_swerve_mpc_planner",
        executable="swerve_animator_node",
        output="screen",
        parameters=[config_file],
    )
    return LaunchDescription([swerve_animator_node, plotjuggler])
