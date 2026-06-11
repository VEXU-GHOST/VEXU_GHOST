#!/bin/bash
set -x
logger "RUNNING launch_hardware.sh"

cd
export VEXU_HOME="/home/ghost/VEXU_GHOST"
source "$VEXU_HOME/scripts/setup_env.sh"

echo "Killing existing"
"$VEXU_HOME/scripts/ghost" kill

ROBOT_NAME="$(cat /etc/ghost/robot_name 2>/dev/null)"

# log to stdout AND /var/log/syslog
if [ -z "$ROBOT_NAME" ];
then
    echo "robot name is unset (/etc/ghost/robot_name)... exiting";
    # ros2 launch ghost_push_back hardware.launch.py 2>&1 | tee /dev/tty |& logger;
else echo "robot name is set to '$ROBOT_NAME'";
    ros2 launch ghost_push_back hardware.launch.py robot_name:=$ROBOT_NAME 2>&1 | tee /dev/tty |& logger;
fi

logger "RUNNING ros2 launch ghost_push_back DONE"
logger ${PIPESTATUS}
