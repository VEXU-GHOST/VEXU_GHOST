#!/bin/bash
set -x
logger "RUNNING launch_hardware.sh"

cd
export VEXU_HOME="/home/ghost/VEXU_GHOST"
source "$VEXU_HOME/scripts/setup_env.sh"

# log to stdout AND /var/log/syslog
if [ -z ${ROBOT_NAME+x} ]; 
then 
    echo "var is unset"; 
    ros2 launch ghost_high_stakes hardware.launch.py 2>&1 | tee /dev/tty |& logger;
else echo "var is set to '$var'";
    ros2 launch ghost_high_stakes alpha_jerry.launch.py 2>&1 | tee /dev/tty |& logger;
fi

logger "RUNNING ros2 launch ghost_high_stakes DONE"
logger ${PIPESTATUS}
