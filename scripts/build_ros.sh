#!/bin/bash

# Assumes repository is in base directory
cd ~/VEXU_GHOST
echo "---Building Ghost ROS Packages---"

# Build simulator packages depending on what is passed for EMBEDBUILD
if [ "$1" == "EMBEDBUILD" ];
then 
    colcon build --packages-up-to ghost_ros
else
    colcon build
fi

source install/setup.bash

bash scripts/generate_urdfs.sh

