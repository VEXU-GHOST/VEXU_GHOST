# TEMP: V5 serial node + foxglove bridge only.
# Picks the robot the same way the PROS header generator does, so brain and
# serial node always agree: $VEXU_HOME/robotname (default EXAMPLE_ROBOT) -> robots.yaml.
import os
import glob
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The checkout this file lives in (not $VEXU_HOME, which may point at another clone).
    repo = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", ".."))

    try:
        with open(os.path.join(repo, "robotname")) as f:
            robot = f.read().strip().upper()
    except FileNotFoundError:
        robot = ""
    robot = robot or "EXAMPLE_ROBOT"

    with open(os.path.join(repo, "robots.yaml")) as f:
        hw_config = os.path.join(repo, yaml.safe_load(f)[robot]["config_file"])

    # Per-robot ROS config sits next to the hardware config (x_hardware_config -> x_ros_config);
    # ghost_override robots also share a base_ros_config.yaml one level up.
    cfg_dir = os.path.dirname(hw_config)
    params = []
    base = os.path.join(cfg_dir, "..", "base_ros_config.yaml")
    if os.path.exists(base):
        params.append(base)
    params.append(os.path.join(cfg_dir, os.path.basename(hw_config).replace("hardware_config", "ros_config")))

    port = (sorted(glob.glob("/dev/serial/by-id/*V5_Brain*-if02")) or ["/dev/ttyACM1"])[0]
    print(f"serial_plot: robot={robot} hw_config={hw_config} port={port}")

    return LaunchDescription([
        Node(
            package="ghost_ros_interfaces",
            executable="jetson_v5_serial_node",
            name="ghost_serial_node",
            output="screen",
            parameters=params + [
                {"robot_config_yaml_path": hw_config},
                {"port_name": port, "backup_port_name": port},
            ],
        ),
        Node(package="foxglove_bridge", executable="foxglove_bridge", parameters=[{"port": 8765}]),
    ])
