#!/bin/bash
pkill -f gz
source  ~/VEXU_GHOST/install/setup.bash
ros2 launch ghost_sim start_sim.launch.py