import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    home_dir = os.path.expanduser("~")
    config_path = os.path.join(home_dir, "VEXU_GHOST", "11_Robots", "ghost_high_stakes", "config")

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(config_path, "alpha/alpha_ros_config.yaml")

    # This contains all the port and device info that gets compiled on to the V5 Brain
    robot_config_yaml_path = os.path.join(config_path, "alpha/alpha_hardware_config.yaml")

    plugin_type = "ghost_tank::AlphaJerryPlugin"
    robot_name = "ALPHA_JERRY"

    ghost_tank_share_dir = get_package_share_directory("ghost_tank")
    bt_path = os.path.join(ghost_tank_share_dir, "config", "bt_isolation_alpha_jerry.xml")
    bt_path_interaction = os.path.join(ghost_tank_share_dir, "config", "bt_interaction.xml")
    config_path = os.path.join(ghost_tank_share_dir, "config")

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
        # arguments=["--ros-args", "--log-level", "debug"]
    )

    competition_state_machine_node = Node(
        package="ghost_ros_interfaces",
        executable="competition_state_machine_node",
        output="screen",
        parameters=[
            ros_config_file,
            {
                "robot_config_yaml_path": robot_config_yaml_path,
                "bt_path": bt_path,
                "bt_path_interaction": bt_path,
                "config_path": config_path,
            },
        ],
        arguments=[plugin_type, robot_name],
        # arguments=["--ros-args", "--log-level", "debug"]
    )

    imu_filter_node = Node(
        package="ghost_sensing",
        executable="imu_filter_node",
        name="imu_filter_node",
        output="screen",
        parameters=[ros_config_file],
    )

    gpio_expander = Node(
        package="ghost_io",
        executable="gpio_expander",
        name="gpio_expander",
        output="screen",
        parameters=[ros_config_file],
    )

    #color_sensor_intake = Node(
    #     package="ghost_sensing",
    #     executable="tcs_color_sensor",
    #     name="tcs_color_sensor_intake",
    #     output="screen",
    #     namespace="/sensors/color_sensors/intake",
    #     parameters=[
    #         ros_config_file
    #         # address 0x29, not configurable on tcs
    #    ],
    # )

    color_classifier_intake = Node(
        package="ghost_sensing",
        executable="color_classifier",
        name="color_classifier_0",
        output="screen",
        namespace="/sensors/color_sensors/intake",
        parameters=[ros_config_file],
    )

    color_sensor_goal_rush_l = Node(
        package="ghost_sensing",
        executable="avago_color_sensor",
        name="avago_color_sensor_goal_rush_l",
        output="screen",
        namespace="/sensors/color_sensors/goal_rush_l",
        parameters=[ros_config_file, {
            "address": 0x39^ (1<<6), # both address translator switches on so ^ 1<<6
        }],
    )
    color_sensor_intake = Node(
        package="ghost_sensing",
        executable="avago_color_sensor",
        name="avago_color_sensor_intake",
        output="screen",
        namespace="/sensors/color_sensors/intake",
        parameters=[ros_config_file, {
            "address": 0x39 , # both address translator switches on so ^ 1<<6
        }],
    )
    # no need for color classifier, since we only use proximity for goal rush


    odom_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_ekf_node",
        output="screen",
        parameters=[ros_config_file],
        remappings=[("odometry/filtered", "/odom_ekf/odometry")],
    )

    map_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="map_ekf_node",
        output="screen",
        parameters=[ros_config_file],
        remappings=[("odometry/filtered", "/map_ekf/odometry")],
    )

    ekf_pf_node = Node(
        package="ghost_localization",
        executable="ekf_pf_node",
        name="ekf_pf_node",
        output="screen",
        parameters=[ros_config_file],
    )

    return LaunchDescription([
        serial_node,
        imu_filter_node,
        # ekf_pf_node,
        # odom_ekf_node,
        # map_ekf_node,
        # color_sensor_intake,
        # color_classifier_intake,
                # color_sensor_goal_rush_l,
                #color_sensor_goal_rush_r,
        competition_state_machine_node,
        # gpio_expander,
    ])

