#!/bin/bash
pkill -f gz
source  ~/VEXU_GHOST/install/setup.bash
ros2 launch ghost_ros swerve.launch.py