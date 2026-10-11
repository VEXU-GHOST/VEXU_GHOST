"""D435 RGB -> image_proc -> upstream apriltag_ros, standalone or on the robot."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _camera_info_topic(image_topic):
    # image_transport derives CameraInfo from the resolved image topic's parent.
    return image_topic.rsplit("/", 1)[0] + "/camera_info"


def _launch_setup(context):
    tag_size = float(LaunchConfiguration("tag_size").perform(context))
    if not math.isfinite(tag_size) or tag_size <= 0:
        raise ValueError("tag_size must be a finite positive edge length in meters")
    use_sim_time = _boolean(context, "use_sim_time")
    start_camera = _boolean(context, "start_camera")
    if start_camera and use_sim_time:
        raise ValueError("For bag replay use start_camera:=false with use_sim_time:=true")

    image_topic = LaunchConfiguration("image_topic").perform(context)
    info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    rect_topic = LaunchConfiguration("rectified_topic").perform(context)
    for topic in (image_topic, info_topic, rect_topic):
        if not topic.startswith("/") or topic.endswith("/"):
            raise ValueError("Camera topic arguments must be absolute ROS topic names")
    if len({image_topic, info_topic, rect_topic}) != 3:
        raise ValueError("Raw image, rectified image, and CameraInfo topics must differ")

    actions = []
    if start_camera:
        # Launch the upstream executable directly: rs_launch.py does not expose
        # color_qos/color_info_qos as launch arguments in wrapper 4.58.4.
        # Explicit matching QoS avoids the Humble rectifier copying transient
        # image durability onto a subscription to volatile CameraInfo.
        actions.append(Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="camera",
            output="screen",
            parameters=[
                LaunchConfiguration("camera_params_file").perform(context),
                {"rgb_camera.color_profile": LaunchConfiguration("color_profile").perform(context)},
            ],
        ))

    actions.append(Node(
        package="image_proc",
        executable="rectify_node",
        name="apriltag_rectify",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("image", image_topic),
            ("camera_info", info_topic),
            (_camera_info_topic(image_topic), info_topic),
            ("image_rect", rect_topic),
        ],
    ))
    actions.append(Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file").perform(context),
            {"family": LaunchConfiguration("family").perform(context),
             "size": tag_size, "use_sim_time": use_sim_time},
        ],
        remappings=[
            ("image_rect", rect_topic),
            ("camera_info", info_topic),
            (_camera_info_topic(rect_topic), info_topic),
            ("detections", "/apriltag/detections"),
        ],
    ))
    return actions


def generate_launch_description():
    # Same relative layout in the source tree and installed package share.
    # Supports `ros2 launch /absolute/path/apriltag.launch.py` without a full build.
    params_file = str(Path(__file__).resolve().parents[1] / "config/apriltag.yaml")
    camera_params_file = str(Path(__file__).resolve().parents[1] / "config/apriltag_camera.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("tag_size", description="Measured tag edge in meters; excludes white margin"),
        DeclareLaunchArgument("family", default_value="36h11"),
        DeclareLaunchArgument("start_camera", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("color_profile", default_value="640x480x30"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("rectified_topic", default_value="/camera/camera/color/image_rect"),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("camera_params_file", default_value=camera_params_file),
        OpaqueFunction(function=_launch_setup),
    ])
