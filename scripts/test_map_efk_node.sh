#!/bin/bash
ros2 run robot_localization ekf_node --ros-args --params-file ~/VEXU_GHOST/11_Robots/ghost_push_back/config/ros_config.yaml -r odometry/filtered:=/map_ekf/odometry