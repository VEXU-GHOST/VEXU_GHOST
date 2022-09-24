#!/bin/bash

cd ~/VEXU_GHOST
colcon build
source install/setup.bash

SHARE_DIR="$PWD/install/ghost_ros/share/ghost_ros"
URDF_PATH="${SHARE_DIR}/urdf/robot_description.urdf"
URDF_PID_PATH="${SHARE_DIR}/urdf/robot_description_pid.urdf"
SDF_PATH="$SHARE_DIR/urdf/swerve.sdf"
SDF_PID_PATH="$SHARE_DIR/urdf/swerve_pid.sdf"

if [ ! -d $SHARE_DIR ];
then
    touch $URDF_PATH
else
    echo
    echo "Generating URDF"
    xacro ghost_ros/urdf/swerve.urdf > $URDF_PATH
    xacro ghost_ros/urdf/swerve_pid.urdf > $URDF_PID_PATH
    gz sdf -p $URDF_PATH > $SDF_PATH
    gz sdf -p $URDF_PID_PATH > $SDF_PID_PATH
    echo "URDF written to" $URDF_PATH
    echo "SDF written to" $SDF_PATH
fi

source ~/.bashrc
