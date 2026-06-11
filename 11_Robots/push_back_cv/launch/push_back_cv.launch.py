import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up the push-back CV perception pipeline.

    Chain (see package diagram):
        realsense2_camera  ->  cv_detector_array  ->  block_map  ->  goal_reader
    The RealSense driver is NOT started here — it belongs to the robot/hardware
    bringup. This launch only owns the three CV nodes so it can be reused from
    hardware.launch.py or run standalone against any source of the camera topics.
    """

    # goal_reader reads goal regions from this YAML. Empty -> the node falls back
    # to its own share dir (config/goal_regions.yaml), so this just lets a caller
    # override it without touching the package install.
    goal_regions_file = LaunchConfiguration("goal_regions_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "goal_regions_file",
            default_value=os.path.join(
                get_package_share_directory("push_back_cv"),
                "config",
                "goal_regions.yaml",
            ),
            description="YAML defining the per-goal field regions for goal_reader.",
        ),

        # 1. Perception: YOLO + aligned depth -> map-frame detections (/cv/detections)
        Node(
            package="push_back_cv",
            executable="cv_detector_array.py",
            name="cv_detector_array",
            output="screen",
        ),

        # 2. Tracking: detections -> persistent field blocks (/field/blocks)
        Node(
            package="push_back_cv",
            executable="block_map_node.py",
            name="block_map",
            output="screen",
        ),

        # 3. Goals: blocks -> per-goal counts + control (/field/goals)
        Node(
            package="push_back_cv",
            executable="goal_reader_node",
            name="goal_reader",
            output="screen",
            parameters=[{"goal_regions_file": goal_regions_file}],
        ),
    ])
