import os
import xacro
from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    ghost_high_stakes_base_dir = os.path.join(os.path.expanduser("~"), "VEXU_GHOST", "11_Robots", "ghost_high_stakes")

    # This contains parameters shared between both robots
    base_ros_config_file = os.path.join(ghost_high_stakes_base_dir, "config/base_ros_config.yaml")
    
    #############################
    ### Base Node Definitions ###
    #############################
    bag_recorder_service = Node(
        package="ghost_ros_interfaces",
        executable="bag_recorder_service",
        output="screen",
        parameters=[base_ros_config_file],
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
            SetRemap(src='/camera/camera/imu', dst='/sensors/imu'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    )
                ),
                launch_arguments={
                    "unite_imu_method": "2",
                    "enable_depth": "false",
                    "enable_color": "false",
                    "enable_sync": "true",
                    "enable_gyro": "true",
                    "enable_accel": "true",
                    "initial_reset": "true",
                    "gyro_fps": "200",  # 200 or 400
                    "accel_fps": "63",  # 63 or 250
                            "color_fps": "1",
                "color_width": "640",
                "color_height": "480",
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
                    os.path.join(ghost_high_stakes_base_dir, "launch", robot_name, robot_name + ".launch.py")
                )
            )
            return [robot_launch]

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="None"),
        rplidar_node,
        realsense_node,
        bag_recorder_service,
        tts_music_node,
        OpaqueFunction(function = launch_setup),
    ])

