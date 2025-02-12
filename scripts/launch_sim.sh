#!/bin/bash
pkill -f gz
source  $VEXU_HOME/install/setup.bash
ros2 launch ghost_sim_examples start_sim.launch.py