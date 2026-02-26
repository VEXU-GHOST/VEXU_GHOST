#!/bin/bash
set -x
logger "RUNNING launch_hardware.sh"

cd
export VEXU_HOME="/home/ghost/VEXU_GHOST"
source "$VEXU_HOME/scripts/setup_env.sh"

# log to stdout AND /var/log/syslog
if [ -z ${ROBOT_NAME+x} ]; 
then 
    echo "ROBOT_NAME is unset... exiting"; 
    # ros2 launch ghost_push_back hardware.launch.py 2>&1 | tee /dev/tty |& logger;
else echo "ROBOT_NAME is set to '$ROBOT_NAME'";
    ros2 launch ghost_push_back hardware.launch.py robot_name:=$ROBOT_NAME 2>&1 | tee /dev/tty |& logger;
fi

logger "RUNNING ros2 launch ghost_push_back DONE"
logger ${PIPESTATUS}
