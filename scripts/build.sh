#!/bin/bash

cd ~/VEXU_GHOST
colcon build
source ~/.bashrc

SHARE_DIR="$PWD/install/swerve_drive/share/swerve_drive"
URDF_PATH="${SHARE_DIR}/urdf/robot_description.urdf"
URDF_PID_PATH="${SHARE_DIR}/urdf/robot_description_pid.urdf"
SDF_PATH="$SHARE_DIR/urdf/swerve.sdf"
SDF_PID_PATH="$SHARE_DIR/urdf/swerve_pid.sdf"

if [ ! -d $URDF_PATH ]; then
    touch $URDF_PATH
fi

echo
echo "Generating URDF"
xacro swerve_drive/urdf/swerve.urdf > $URDF_PATH
xacro swerve_drive/urdf/swerve_pid.urdf > $URDF_PID_PATH
gz sdf -p $URDF_PATH > $SDF_PATH
gz sdf -p $URDF_PID_PATH > $SDF_PID_PATH
echo "URDF written to" $URDF_PATH
echo "SDF written to" $SDF_PATH

source ~/.bashrc
