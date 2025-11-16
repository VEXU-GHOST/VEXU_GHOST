#!/bin/bash
set -x
logger "RUNNING launch_hardware.sh"

cd
export VEXU_HOME="/home/ghost/VEXU_GHOST"
source "$VEXU_HOME/scripts/setup_env.sh"

# log to stdout AND /var/log/syslog
ros2 launch ghost_high_stakes hardware.launch.py 2>&1 | tee /dev/tty |& logger

logger "RUNNING ros2 launch ghost_high_stakes DONE"
logger ${PIPESTATUS}
