import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

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

    # nav2 map server (+ lifecycle manager) that serves the push back field on
    # /map. Defined in ghost_localization alongside the map files.
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ghost_localization"),
                "launch",
                "map_server.launch.py",
            )
        ),
        launch_arguments={"base_params_file": base_ros_config_file}.items(),
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
                    "enable_depth": "false",
                    "enable_color": "false",
                    "enable_gyro": "true",
                    "initial_reset": "false",
                    "gyro_qos": "SENSOR_DATA",
                    "gyro_fps": "200",  # 200 or 400
                }.items(),
            )
        ]
    )

    #######################
    ### Robot Overrides ###
    #######################

    def launch_setup(context, *args, **kwargs): 
        robot_name = LaunchConfiguration("robot_name").perform(context)
        name_options = ["alpha", "omega"]

        if robot_name not in name_options:
            print()
            print("ERROR: Invalid robot_name:", robot_name + "!")
            print("Launching without robot-specific nodes.")
            print()
            return []
        else:
            print("Launching robot_name:", robot_name)
            robot_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ghost_push_back_base_dir, "launch", robot_name, robot_name + ".launch.py")
                ),
                launch_arguments={'base_params_file': base_ros_config_file}.items()
            )
            return [robot_launch]

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="None"),
        robot_state_publisher,
        map_server_launch,
        rplidar_node,
        # realsense_node,
        bag_recorder_service,
        # tts_music_node,
        OpaqueFunction(function = launch_setup),
    ])

