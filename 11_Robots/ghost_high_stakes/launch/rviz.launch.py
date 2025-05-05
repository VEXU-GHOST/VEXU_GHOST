import os
import xacro
import launch
import launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    home_dir = os.path.expanduser("~")
    ghost_high_stakes_base_dir = os.path.join(
        home_dir, "VEXU_GHOST", "11_Robots", "ghost_high_stakes"
    )
    ros_config_file = os.path.join(ghost_high_stakes_base_dir, "config/ros_config.yaml")

    ghost_sim_dir = os.path.join(
        home_dir, "VEXU_GHOST", "04_Sim", "ghost_sim"
    )
    filename = "test_tank_sim_base.xacro"
    xacro_path = os.path.join(ghost_sim_dir, os.path.join("urdf", filename))

    # rviz_config_path = os.path.join(pkg_share, 'rviz/base_link_config.rviz')
    # rviz_config_path = os.path.join(pkg_share, "rviz/odom_config.rviz")
    # rviz_config_path = os.path.join(pkg_share, "rviz/world_config.rviz")

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", xacro_path]),
                "ignore_timestamp": True,
            }
        ],
    )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_path],
    # )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # covariance_2d_publisher_node = Node(
    #     package="ghost_localization",
    #     executable="covariance_2d_publisher",
    #     output="screen",
    #     parameters=[ros_config_file],
    # )

    return launch.LaunchDescription(
        [
            # robot_state_publisher_node,
            # covariance_2d_publisher_node,
            # joint_state_publisher_gui_node,
            # rviz_node,
        ]
    )
