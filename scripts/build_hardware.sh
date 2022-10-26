#!/bin/bash

cd ~/VEXU_GHOST
colcon build --packages-select ghost_ros
source install/setup.bash

SHARE_DIR="$PWD/install/ghost_ros/share/ghost_ros"
URDF_PATH="${SHARE_DIR}/urdf/ghost1.urdf"

if [ ! -d $SHARE_DIR ];
then
    touch $URDF_PATH
    # touch $URDF_PID_PATH
else
    echo
    echo "Generating URDF"
    xacro ghost_ros/urdf/ghost1.xacro > $URDF_PATH
    echo "URDF written to" $URDF_PATH
fi

source ~/.bashrc
