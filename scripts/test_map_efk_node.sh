#!/bin/bash
ros2 run robot_localization ekf_node --ros-args --params-file /home/ghost/VEXU_GHOST/11_Robots/ghost_high_stakes/config/ros_config.yaml -r odometry/filtered:=/map_ekf/odometry