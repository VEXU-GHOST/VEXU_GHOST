#!/bin/bash
# Launch the CV test simulator (Gazebo field + looking-around camera + model scoring).
# Extra args are passed to ros2 launch, e.g.:  ./scripts/launch_cv_sim.sh backend:=color_blob mode:=random
# Live results: http://localhost:8090
if [ -z "${VEXU_IN_DOCKER}" ]; then
	pkill -f gz
fi
source "$VEXU_HOME/install/setup.bash"
ros2 launch ghost_cv_sim cv_sim.launch.py "$@"
