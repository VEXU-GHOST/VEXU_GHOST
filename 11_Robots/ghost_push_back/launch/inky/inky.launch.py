import os
import xacro
import yaml
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Get base params from parent launch file and config path
    base_params_file = LaunchConfiguration("base_params_file")
    # Per-match init settings (alliance colour, etc.) from hardware.launch.py.
    init_config_file = LaunchConfiguration("init_config_file")
    config_path = os.path.join(os.path.expanduser("~"), "VEXU_GHOST", "11_Robots", "ghost_push_back", "config")
    tank_config_path = os.path.join(os.path.expanduser("~"), "VEXU_GHOST", "11_Robots", "ghost_tank", "config")

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(config_path, "inky/inky_ros_config.yaml")

    # This contains all the port and device info that gets compiled on to the V5 Brain
    robot_config_yaml_path = os.path.join(config_path, "inky/inky_hardware_config.yaml")

    # Shared localization config (particle filter + robot_localization EKFs),
    # split out of base_ros_config.yaml.

    # This specifies robot control plugin yo load
    plugin_type = "ghost_tank::InkyPlugin"
    robot_name = "INKY"

    # Get BT Path for autons. The BT filenames live in this robot's init config
    # so they can be swapped per match without touching this launch file; we
    # resolve them to absolute paths under the ghost_tank share config dir here.
    ghost_tank_share_dir = get_package_share_directory("ghost_tank")
    init_config_path = os.path.join(config_path, "inky/inky_init_config.yaml")
    with open(init_config_path, "r") as f:
        init_config = yaml.safe_load(f)
    csm_params = init_config["competition_state_machine_node"]["ros__parameters"]
    bt_path = os.path.join(ghost_tank_share_dir, "config", csm_params["bt_filename"])
    bt_path_interaction = os.path.join(
        ghost_tank_share_dir, "config", csm_params["bt_filename_interaction"])

    ########################
    ### Node Definitions ###
    ########################
    # V5 brain serial port: auto-detected by hardware.launch.py (USB id, via
    # /dev/serial/by-id) and passed in here, overriding the base_ros_config
    # fallback. Defaults to the fallback when this launch is run standalone.
    v5_serial_port = LaunchConfiguration("v5_serial_port")

    serial_node = Node(
        package="ghost_ros_interfaces",
        executable="jetson_v5_serial_node",
        name="ghost_serial_node",
        output="screen",
        parameters=[
            base_params_file,
            ros_config_file,
            {"robot_config_yaml_path": robot_config_yaml_path},
            {"port_name": v5_serial_port, "backup_port_name": v5_serial_port},
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
            init_config_file,
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

    # Ball colour classifiers are launched by hardware.launch.py (grouped,
    # one per COLOR device in the sensor host config).


    def localization_setup(context, *args, **kwargs):
        use_dev = LaunchConfiguration("use_dev_field").perform(context) == "true"
        if use_dev:
            loc_config = os.path.join(config_path, "localization_config_dev.yaml")
            pf_extra = [{"particle_filter.map": os.path.join(
                get_package_share_directory("ghost_localization"), "maps", "devFieldZeroLH.txt"
            )}]
        else:
            loc_config = os.path.join(config_path, "localization_config.yaml")
            pf_extra = []
        return [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="odom_ekf_node",
                output="screen",
                parameters=[ros_config_file, base_params_file, loc_config],
                remappings=[("odometry/filtered", "/odom_ekf/odometry")],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="map_ekf_node",
                output="screen",
                parameters=[ros_config_file, base_params_file, loc_config],
                remappings=[("odometry/filtered", "/map_ekf/odometry")],
            ),
            Node(
                package="ghost_localization",
                executable="ekf_pf_node",
                name="ekf_pf_node",
                output="screen",
                parameters=[ros_config_file, base_params_file, loc_config] + pf_extra,
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('base_params_file'),
        DeclareLaunchArgument('init_config_file'),
        DeclareLaunchArgument('v5_serial_port', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('use_dev_field', default_value='false'),
        serial_node,
        imu_filter_node,
        competition_state_machine_node,
        # gpio_expander,
        OpaqueFunction(function=localization_setup),
    ])

