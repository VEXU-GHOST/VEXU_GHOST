#!/bin/bash
if [ -z "${VEXU_IN_DOCKER}" ]; then
	pkill -f gz
fi
INSTALL_BASE="${VEXU_COLCON_INSTALL_BASE:-install}"
source "$VEXU_HOME/${INSTALL_BASE}/setup.bash"
ros2 launch ghost_sim_examples start_sim.launch.py