import os

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    ghost_over_under_base_dir = os.path.join(home_dir, "VEXU_GHOST", "11_Robots", "ghost_over_under")

    # This contains all the parameters for our ROS nodes
    ros_config_file = os.path.join(ghost_over_under_base_dir, "config/ros_config.yaml")

    # This contains all the port and device info that gets compiled on to the V5 Brain
    robot_config_yaml_path = os.path.join(ghost_over_under_base_dir, "config/robot_hardware_config.yaml")

    plugin_type = "ghost_swerve::SwerveRobotPlugin"
    robot_name = "ghost_15"

    ghost_autonomy_share_dir = get_package_share_directory('ghost_autonomy')
    bt_path = os.path.join(ghost_autonomy_share_dir, "config", "bt.xml")
    

    ########################
    ### Node Definitions ###
    ########################
    serial_node = Node(
        package='ghost_ros_interfaces',
        executable='jetson_v5_serial_node',
        name='ghost_serial_node',
        output='screen',
        parameters=[ros_config_file, {"robot_config_yaml_path": robot_config_yaml_path}],
        # arguments=["--ros-args", "--log-level", "debug"]
    )

    competition_state_machine_node = Node(
        package='ghost_ros_interfaces',
        executable='competition_state_machine_node',
        output='screen',
        parameters=[ros_config_file, 
                    {
                        "robot_config_yaml_path": robot_config_yaml_path,
                        "bt_path": bt_path
                    }],
        arguments=[plugin_type, robot_name]
        # arguments=["--ros-args", "--log-level", "debug"]
    )

    swerve_motion_planner_node = Node(
        package='ghost_swerve',
        executable='swerve_motion_planner',
        name='motion_planner',
        output='screen',
        parameters=[ros_config_file, 
                    {
                        "robot_config_yaml_path": robot_config_yaml_path
                    }],
        arguments=[plugin_type, robot_name]
        # arguments=["--ros-args", "--log-level", "debug"]
    )

    # rplidar_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_scan_publisher',
    #     name='rplidar_scan_publisher',
    #     parameters=[{"frame_id": "lidar_link", 'angle_compensate': True}]
    # )

    imu_filter_node = Node(
        package='ghost_sensing',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[ros_config_file],
    )

    realsense_share = get_package_share_directory('realsense2_camera')
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share,
                         'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'unite_imu_method': '2',
            'enable_depth': 'false',
            'enable_color': 'false',
            'enable_sync': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'gyro_fps': '200', # 200 or 400
            'accel_fps': '63', # 63 or 250
            }.items()
    )

    return LaunchDescription([
        serial_node,
        competition_state_machine_node,
        realsense_node,
        imu_filter_node
        # swerve_motion_planner_node
        # rplidar_node,
    ])