#!/bin/bash
set -x
logger "RUNNING launch_hardware.sh"

cd
export VEXU_HOME="/home/ghost/VEXU_GHOST"
source "$VEXU_HOME/scripts/setup_env.sh"

# Skip when launched by systemd (systemd sets INVOCATION_ID); killing here
# would tear down the very service that started us.
if [ -z "$INVOCATION_ID" ]; then
    echo "Killing existing"
    "$VEXU_HOME/scripts/ghost" kill
else
    echo "Running under systemd ($INVOCATION_ID), skipping kill"
fi

ROBOT_NAME="$(tr -d '[:space:]' < "$VEXU_HOME/robotname" 2>/dev/null | tr 'A-Z' 'a-z')"

if [ -z "$ROBOT_NAME" ]; then
    ROBOT_NAME="example_robot"
    RED=$'\033[1;31m'; RESET=$'\033[0m'
    MSG="WARNING: NO ROBOT NAME SET in $VEXU_HOME/robotname! Defaulting to '$ROBOT_NAME'. Run: ghost set-robot-name <name>"
    echo "${RED}########################################################################${RESET}"
    echo "${RED}${MSG}${RESET}"
    echo "${RED}########################################################################${RESET}"
    logger "$MSG"
fi

# log to stdout AND /var/log/syslog
echo "robot name is set to '$ROBOT_NAME'"
ros2 launch ghost_override hardware.launch.py robot_name:=$ROBOT_NAME 2>&1 | tee /dev/tty |& logger

logger "RUNNING ros2 launch ghost_override DONE"
logger ${PIPESTATUS}
