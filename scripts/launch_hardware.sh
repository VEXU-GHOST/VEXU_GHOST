#!/bin/bash
logger "RUNNING launch_hardware.sh"
cd ~/VEXU_GHOST
source ~/.bashrc

# idk if we need to double it given it's already in bashrc but it works so don't touch it
source /opt/ros/humble/setup.bash
source ~/VEXU_GHOST/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
export GAZEBO_PLUGIN_PATH=$HOME/VEXU_GHOST/build/ghost_sim:$GAZEBO_PLUGIN_PATH

# log to stdout AND /var/log/syslog
ros2 launch ghost_over_under hardware.launch.py 2>&1 | tee /dev/tty |& logger

logger "RUNNING ros2 launch ghost_over_under DONE"
logger ${PIPESTATUS}
