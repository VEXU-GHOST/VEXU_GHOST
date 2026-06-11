import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Get base params from parent launch file and config path
    base_params_file = LaunchConfiguration("base_params_file")
    config_path = os.path.join(os.path.expanduser("~"), "VEXU_GHOST", "11_Robots", "ghost_high_stakes", "config")
    tank_config_path = os.path.join(os.path.expanduser("~"), "VEXU_GHOST", "11_Robots", "ghost_tank", "config")

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(config_path, "omega/omega_ros_config.yaml")

    # This contains all the port and device info that gets compiled on to the V5 Brain
    robot_config_yaml_path = os.path.join(config_path, "omega/omega_hardware_config.yaml")

    # This specifies robot control plugin yo load
    plugin_type = "ghost_tank::OmegaJerryPlugin"
    robot_name = "OMEGA_JERRY"

    # Get BT Path for autons
    ghost_tank_share_dir = get_package_share_directory("ghost_tank")
    bt_path = os.path.join(ghost_tank_share_dir, "config", "bt_isolation.xml")
    bt_path_interaction = os.path.join(ghost_tank_share_dir, "config", "bt_interaction.xml")

    ########################
    ### Node Definitions ###
    ########################
    serial_node = Node(
        package="ghost_ros_interfaces",
        executable="jetson_v5_serial_node",
        name="ghost_serial_node",
        output="screen",
        parameters=[
            base_params_file,
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
            base_params_file,
            ros_config_file,
            {
                "robot_config_yaml_path": robot_config_yaml_path,
                "bt_path": bt_path,
                "bt_path_interaction": bt_path_interaction,
                "config_path": tank_config_path,
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
        parameters=[ros_config_file, base_params_file],
    )

    gpio_expander = Node(
        package="ghost_io",
        executable="gpio_expander",
        name="gpio_expander",
        output="screen",
        parameters=[ros_config_file, base_params_file],
    )

    color_sensor_intake = Node(
        package="ghost_sensing",
        executable="tcs_color_sensor",
        name="tcs_color_sensor_intake",
        output="screen",
        namespace="/sensors/color_sensors/intake",
        parameters=[
            ros_config_file, base_params_file
            # address 0x29, not configurable on tcs
       ],
    )

    color_classifier_intake = Node(
        package="ghost_sensing",
        executable="color_classifier",
        name="color_classifier_0",
        output="screen",
        namespace="/sensors/color_sensors/intake",
        parameters=[ros_config_file, base_params_file],
    )

    color_sensor_goal_rush_l = Node(
        package="ghost_sensing",
        executable="avago_color_sensor",
        name="avago_color_sensor_goal_rush_l",
        output="screen",
        namespace="/sensors/color_sensors/goal_rush_l",
        parameters=[ros_config_file, base_params_file, {
            "address": 0x69, # both address translator switches off so ^ 0x70
        }],
    )
    color_sensor_goal_rush_r = Node(
        package="ghost_sensing",
        executable="avago_color_sensor",
        name="avago_color_sensor_goal_rush_r",
        output="screen",
        namespace="/sensors/color_sensors/goal_rush_r",
        parameters=[ros_config_file, base_params_file, {
            "address": 0x99, # one switch on idk which trial and error so ^ 0x40
        }],
    )
    color_sensor_goal_clamp = Node(
        package="ghost_sensing",
        executable="avago_color_sensor",
        name="avago_color_sensor_goal_clamp",
        output="screen",
        namespace="/sensors/color_sensors/goal_clamp",
        parameters=[ros_config_file, base_params_file, {
            "address": 0x39, # one switch on idk which trial and error so ^ 0x40
        }],
    )


    odom_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_ekf_node",
        output="screen",
        parameters=[ros_config_file, base_params_file],
        remappings=[("odometry/filtered", "/odom_ekf/odometry")],
    )

    map_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="map_ekf_node",
        output="screen",
        parameters=[ros_config_file, base_params_file],
        remappings=[("odometry/filtered", "/map_ekf/odometry")],
    )

    ekf_pf_node = Node(
        package="ghost_localization",
        executable="ekf_pf_node",
        name="ekf_pf_node",
        output="screen",
        parameters=[ros_config_file, base_params_file],
    )

    partner_pose_publisher_node = Node(
        package="ghost_ros_interfaces",
        executable="partner_pose_publisher_node",
        name="partner_pose_publisher_node",
        output="screen",
        parameters=[ros_config_file, base_params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument('base_params_file'),
        serial_node,
        imu_filter_node,
        odom_ekf_node,
        ekf_pf_node,
        map_ekf_node,
        color_sensor_intake,
        color_classifier_intake,
        # color_sensor_goal_rush_l,
        # color_sensor_goal_rush_r,
        color_sensor_goal_clamp,

        competition_state_machine_node,
        gpio_expander,
        partner_pose_publisher_node,
    ])

