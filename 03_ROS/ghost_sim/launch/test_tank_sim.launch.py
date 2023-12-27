import os
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Opaque Function hack to allow for better CLI arg parsing
def launch_setup(context, *args, **kwargs):
    ghost_sim_share_dir = get_package_share_directory('ghost_sim')
    filename = "test_tank_init.urdf"

    # Load URDF and process to text
    urdf_path = os.path.join(ghost_sim_share_dir, "urdf", filename)
    doc = xacro.process(urdf_path)
    
    spawn_entity_args = ("-x 0.0 -y 0.0 -z 1.0 -R 0.0 -P 0.0 -Y 0.0 -entity ghost1 -topic robot_description").split()

    # Node to spawn robot model in Gazebo
    gazebo_ros = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        output='screen',
        arguments=spawn_entity_args)

    # Node to publish robot joint transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, {"robot_description": doc}])

    # Joystick (Only launched if joystick CLI arg is set to True)
    joy_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ghost_sim_share_dir,
                "launch",
                "joystick.launch.py"
            )
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_joy")),
        launch_arguments={
            'channel_id': LaunchConfiguration("channel_id"),
        }.items()
    )

    return [gazebo_ros, robot_state_publisher, joy_launch_description]


def generate_launch_description():
    # Load relevant filepaths
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    ghost_ros_share_dir = get_package_share_directory('ghost_ros')
    ghost_sim_share_dir = get_package_share_directory('ghost_sim')

    home_dir = os.path.expanduser('~')
    ghost_ros_base_dir = os.path.join(home_dir, "VEXU_GHOST", "03_ROS", "ghost_ros")

    world_file = os.path.join(ghost_sim_share_dir, "urdf", "spin_up.world")
    rviz_config_path = os.path.join(ghost_ros_share_dir, 'rviz/urdf_config.rviz')

    # Simulator (Doesn't launch Simulator GUI by default, use CLI Arg "sim_gui" for debugging)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir,
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration("sim_gui"),
            'verbose': LaunchConfiguration('verbose'),
            }.items()
    )

    ground_truth_publisher = Node(
        package='ghost_sim',
        executable='ground_truth_pose_publisher',
        name='ground_truth_pose_publisher',
    )

    v5_actuator_cmd_publisher = Node(
        package='ghost_sim',
        executable = 'test_publisher_v5_actuator_cmd',
        name = 'test_publisher_v5_actuator_cmd',
    )

    # Launch RVIZ Display as primary GUI interface
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[ghost_ros_base_dir + "/config/robot_localization_config.yaml"]
    )

    state_machine_node = Node(
        package='ghost_ros',
        executable='robot_state_machine_node',
        name='ghost_state_machine',
        output='screen',
        parameters=[ghost_ros_base_dir + "/config/ghost_state_machine_config.yaml"]
    )

    plot_juggler_node = Node(
        package = 'plotjuggler',
        executable = 'plotjuggler',
        name='plot_juggler'       
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_joy', default_value='false'),
        DeclareLaunchArgument(name='channel_id', default_value='1'),
        DeclareLaunchArgument('sim_gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        simulation,
        # ground_truth_publisher,
        rviz_node,
        # plot_juggler_node,
        # robot_localization_node,
        # state_machine_node,
        # v5_actuator_cmd_publisher,
        OpaqueFunction(function = launch_setup)
    ])