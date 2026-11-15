"""CV test simulator: Gazebo field + free camera + detector + scoring.

  ros2 launch ghost_cv_sim cv_sim.launch.py                       # step through views, Roboflow model
  ros2 launch ghost_cv_sim cv_sim.launch.py backend:=color_blob   # no key/weights needed (smoke test)
  ros2 launch ghost_cv_sim cv_sim.launch.py mode:=random num_scenes:=20 views_per_scene:=10
      -> batch run: scores 200 views over 20 random scenes, writes a report, exits.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("ghost_cv_sim")
    gazebo_share = get_package_share_directory("gazebo_ros")
    cfg = os.path.join(share, "config")
    L = LaunchConfiguration

    def arg(name, default, doc):
        return DeclareLaunchArgument(name, default_value=default, description=doc)

    def val(name, typ):
        return ParameterValue(L(name), value_type=typ)

    args = [
        arg("mode", "step", "camera mode: step | random | sweep | manual"),
        arg("backend", "roboflow", "detector: roboflow | ultralytics | custom | color_blob"),
        arg("scenario", "default", "file in config/scenarios/ (without .yaml)"),
        arg("objects_file", os.path.join(cfg, "objects.yaml"), "object catalog"),
        arg("num_scenes", "0", "stop + write report after N scenes (0 = run forever)"),
        arg("views_per_scene", "0", "views before re-randomizing (0 = auto)"),
        arg("seed", "-1", "RNG seed (-1 = random each run)"),
        arg("class_agnostic", "false", "score detections without checking class names"),
        arg("model_id", "ras-yolo/vex-override-w4dvt-4-yolo26m-t1", "roboflow model id"),
        arg("weights", "", "ultralytics weights path"),
        arg("custom_callable", "", "custom detector, 'package.module:function'"),
        arg("web_port", "8090", "browser viewer port (0 = off)"),
        arg("report_dir", "", "where reports go (default $VEXU_HOME/cv_sim_reports)"),
        arg("gui", "false", "open the Gazebo client window"),
        arg("verbose", "false", "verbose gazebo output"),
    ]

    sim_time = {"use_sim_time": True}
    params_file = os.path.join(cfg, "cv_sim.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"world": os.path.join(share, "worlds", "cv_field.world"),
                          "gui": L("gui"), "verbose": L("verbose")}.items())

    scene_manager = Node(
        package="ghost_cv_sim", executable="scene_manager", name="scene_manager", output="screen",
        parameters=[params_file, sim_time, {
            "objects_file": val("objects_file", str),
            "scenario_file": ParameterValue([share + "/config/scenarios/", L("scenario"), ".yaml"],
                                            value_type=str),
            "seed": val("seed", int)}])

    scanner = Node(
        package="ghost_cv_sim", executable="camera_scanner", name="camera_scanner", output="screen",
        parameters=[params_file, sim_time, {
            "mode": val("mode", str), "num_scenes": val("num_scenes", int),
            "views_per_scene": val("views_per_scene", int), "seed": val("seed", int)}])

    detector = Node(
        package="ghost_cv_sim", executable="cv_detector", name="cv_detector", output="screen",
        parameters=[params_file, sim_time, {
            "backend": val("backend", str), "model_id": val("model_id", str),
            "weights": val("weights", str), "custom_callable": val("custom_callable", str)}])

    evaluator = Node(
        package="ghost_cv_sim", executable="cv_evaluator", name="cv_evaluator", output="screen",
        parameters=[params_file, sim_time, {
            "objects_file": val("objects_file", str), "class_agnostic": val("class_agnostic", bool),
            "web_port": val("web_port", int), "report_dir": val("report_dir", str)}])

    # A batch run (num_scenes > 0) ends when the scanner has saved the report.
    finish = RegisterEventHandler(
        OnProcessExit(target_action=scanner, on_exit=[EmitEvent(event=Shutdown(reason="batch finished"))]),
        condition=IfCondition(PythonExpression(["int('", L("num_scenes"), "') > 0"])))

    return LaunchDescription(args + [gazebo, scene_manager, evaluator, detector, scanner, finish])
