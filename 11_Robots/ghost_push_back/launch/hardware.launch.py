import os
import glob
import xacro
import yaml
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def resolve_serial(pattern, fallback):
    """Resolve a USB serial device by its stable /dev/serial/by-id name.

    by-id symlinks are created by standard udev on any Linux (no custom rules),
    and the name encodes vendor/product/interface, so a glob matches a device by
    TYPE regardless of which /dev/ttyACMx it landed on or its per-board serial.
    Returns the by-id path (stable; safe to open directly) or `fallback` if no
    device matches (e.g. unplugged, or /dev/serial/by-id absent).
    """
    hits = sorted(glob.glob("/dev/serial/by-id/" + pattern))
    return hits[0] if hits else fallback


def generate_launch_description():
    ghost_push_back_base_dir = os.path.join(os.path.expanduser("~"), "VEXU_GHOST", "11_Robots", "ghost_push_back")

    # This contains parameters shared between both robots
    base_ros_config_file = os.path.join(ghost_push_back_base_dir, "config/base_ros_config.yaml")


    #############################
    ### Base Node Definitions ###
    #############################
    bag_recorder_service = Node(
        package="ghost_ros_interfaces",
        executable="bag_recorder_service",
        output="screen",
        parameters=[base_ros_config_file],
    )

    # Inter-robot comms: publishes this robot's quantized map->base_link pose + status on /comms/self
    # (the V5 serial node relays it to the peer over VEXlink), and de-quantizes the peer's relayed
    # state from /comms/other_robot into a map -> other_robot/base_link TF for rviz / costmaps / tf.
    inter_robot_comms_node = Node(
        package="ghost_ros_interfaces",
        executable="inter_robot_comms_node",
        name="inter_robot_comms_node",
        output="screen",
        parameters=[base_ros_config_file],
    )

    # Publish the robot transform tree (base_link -> sensor frames) from the URDF
    # so rviz, costmaps, and the particle filter share one source of truth for
    # sensor poses.
    urdf_path = os.path.join(ghost_push_back_base_dir, "urdf", "ghost_push_back.urdf.xacro")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": xacro.process_file(urdf_path).toxml()}],
    )

    def field_setup(context, *args, **kwargs):
        use_dev = LaunchConfiguration("use_dev_field").perform(context) == "true"
        maps_dir = os.path.join(get_package_share_directory("ghost_localization"), "maps")
        if use_dev:
            nav2_cfg = os.path.join(ghost_push_back_base_dir, "config/nav2_config_dev.yaml")
            map_yaml = os.path.join(maps_dir, "devFieldMapConfig.yaml")
            goal_regions = os.path.join(
                get_package_share_directory("push_back_cv"), "config", "goal_regions_dev.yaml"
            )
        else:
            nav2_cfg = os.path.join(ghost_push_back_base_dir, "config/nav2_config.yaml")
            map_yaml = os.path.join(maps_dir, "pushBackMapConfig.yaml")
            goal_regions = os.path.join(
                get_package_share_directory("push_back_cv"), "config", "goal_regions.yaml"
            )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("ghost_localization"),
                    "launch", "map_server.launch.py",
                )),
                launch_arguments={
                    "base_params_file": nav2_cfg,
                    "yaml_filename": map_yaml,
                }.items(),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_cfg],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_cfg],
                remappings=[("cmd_vel", "/nav2/cmd_vel")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("push_back_cv"),
                    "launch", "push_back_cv.launch.py",
                )),
                launch_arguments={"goal_regions_file": goal_regions}.items(),
            ),
        ]

    # Single lifecycle manager that configures + activates both nav2 servers on
    # startup, in order (planner before controller).
    planner_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["planner_server", "controller_server"]}],
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

    tts_music_node = Node(
        package="ghost_io_py",
        executable="ghost_tts",
        name="tts_music_node",
        output="screen",
        parameters=[base_ros_config_file],
    )

    realsense_node = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    )
                ),
                launch_arguments={
                    "enable_depth": "true",
                    "enable_color": "true",
                    # Shrink streams to their smallest 15 FPS profiles to fit
                    # USB bandwidth. Depth and color expose different resolution
                    # ladders (depth floor 480x270, color floor 424x240);
                    # align reprojects depth into the color frame regardless.
                    "depth_module.depth_profile": "480x270x15",
                    "rgb_camera.color_profile": "424x240x15",
                    # CV subscribes to /camera/.../aligned_depth_to_color/image_raw,
                    # so the depth stream must be registered into the color frame.
                    "align_depth.enable": "true",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "initial_reset": "false",
                }.items(),
            )
        ]
    )


    #######################
    ### Robot Overrides ###
    #######################

    def launch_setup(context, *args, **kwargs): 
        robot_name = LaunchConfiguration("robot_name").perform(context)
        name_options = ["pinky", "inky"]

        if robot_name not in name_options:
            print()
            print("ERROR: Invalid robot_name:", robot_name + "!")
            print("Launching without robot-specific nodes.")
            print()
            return []
        else:
            print("Launching robot_name:", robot_name)

            # Resolve serial devices by USB type via /dev/serial/by-id (standard
            # udev, no custom rules). The V5 brain exposes two interfaces; if02 is
            # the comms "User Port". Fallbacks apply only if by-id is unavailable.
            v5_serial_port = resolve_serial("*V5_Brain*-if02", "/dev/ttyACM1")
            sensor_host_serial_port = resolve_serial(
                "usb-Raspberry_Pi_Pico*-if00", "/dev/ttyACM0")

            # Per-match init settings (alliance colour, auton BT, etc.) for the
            # competition state machine. Per-robot; threaded down to the
            # per-robot launch -> competition_state_machine_node.
            init_config_file = os.path.join(
                ghost_push_back_base_dir, "config", robot_name, robot_name + "_init_config.yaml")

            robot_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ghost_push_back_base_dir, "launch", robot_name, robot_name + ".launch.py")
                ),
                launch_arguments={
                    'base_params_file': base_ros_config_file,
                    'init_config_file': init_config_file,
                    'v5_serial_port': v5_serial_port,
                    'use_dev_field': LaunchConfiguration("use_dev_field").perform(context),
                }.items()
            )

            # RP2040 sensor host. ros_config has namespace; serial_port is the
            # auto-detected Pico path (overrides the base fallback). The device
            # map (what sensors, where) is the per-robot sensor host yaml.
            ros_config_file = os.path.join(
                ghost_push_back_base_dir, "config", robot_name, robot_name + "_ros_config.yaml")
            sensor_host_config = os.path.join(
                ghost_push_back_base_dir, "config", robot_name, robot_name + "_sensor_host_config.yaml")
            sensor_host_node = Node(
                package="ghost_ros_interfaces",
                executable="jetson_sensor_host_serial_node",
                name="ghost_sensor_host_serial_node",
                output="screen",
                parameters=[base_ros_config_file, ros_config_file,
                            {"device_config": sensor_host_config,
                             "serial_port": sensor_host_serial_port}],
            )

            # One ball colour classifier per COLOR device in the sensor host
            # config, grouped together. Each publishes on
            # /sensors/color/<name>/class. Thresholds default to the shared
            # values in base_ros_config (/** section); a COLOR device may set its
            # own red_rb / blue_rb / red_min_level / blue_min_level inline to
            # override them per sensor.
            classifier_nodes = []
            try:
                with open(sensor_host_config) as f:
                    devices = (yaml.safe_load(f) or {}).get("devices") or {}
                for dev_name, dev in devices.items():
                    if str(dev.get("type", "")).upper() == "COLOR":
                        overrides = {"input_topic": f"/sensors/color/{dev_name}"}
                        for key in ("red_rb", "blue_rb", "red_min_level", "blue_min_level"):
                            if key in dev:
                                overrides[key] = dev[key]
                        classifier_nodes.append(Node(
                            package="ghost_sensing",
                            executable="ball_color_classifier",
                            name=f"{dev_name}_color_classifier",
                            output="screen",
                            parameters=[base_ros_config_file, overrides],
                        ))
            except FileNotFoundError:
                print("sensor host config not found:", sensor_host_config)
            color_classifiers = GroupAction(classifier_nodes)

            return [robot_launch, sensor_host_node, color_classifiers]

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="None"),
        DeclareLaunchArgument("use_dev_field", default_value="false",
                              description="Set to 'true' to use dev enclosure configs (8x12 ft, center goal only)."),
        robot_state_publisher,
        planner_lifecycle_manager,
        rplidar_node,
        realsense_node,
        bag_recorder_service,
        inter_robot_comms_node,
        # tts_music_node,
        OpaqueFunction(function=field_setup),
        OpaqueFunction(function=launch_setup),
    ])

