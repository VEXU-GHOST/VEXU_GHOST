import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Serves the push back field occupancy grid on /map via the nav2 map server.
# map_server is a lifecycle node, so the lifecycle manager configures and
# activates it on startup. The map path (yaml_filename) comes from the
# map_server block in nav2_config.yaml, passed in as base_params_file.
# Include this from a robot's hardware launch file.


def generate_launch_description():
    base_params_file = LaunchConfiguration("base_params_file")
    yaml_filename = LaunchConfiguration("yaml_filename")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[base_params_file, {"yaml_filename": yaml_filename}],
    )

    map_server_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )

    return LaunchDescription(
        [
            # Defaults to the push back robot config for standalone launching;
            # hardware.launch.py overrides this with its own base_params_file.
            DeclareLaunchArgument(
                "base_params_file",
                default_value=os.path.join(
                    os.path.expanduser("~"), "VEXU_GHOST", "11_Robots",
                    "ghost_push_back", "config", "nav2_config.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "yaml_filename",
                default_value=os.path.join(
                    get_package_share_directory("ghost_localization"),
                    "maps", "pushBackMapConfig.yaml",
                ),
                description="Absolute path to the occupancy grid YAML passed to map_server.",
            ),
            map_server,
            map_server_lifecycle_manager,
        ]
    )
