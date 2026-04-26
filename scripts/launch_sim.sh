#!/bin/bash
if [ -z "${VEXU_IN_DOCKER}" ]; then
	pkill -f gz
fi
source "$VEXU_HOME/install/setup.bash"
ros2 launch ghost_sim_examples start_sim.launch.py