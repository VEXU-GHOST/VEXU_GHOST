#!/bin/bash

cd ~/VEXU_GHOST
colcon build

SHARE_DIR="$PWD/install/swerve_drive/share/swerve_drive"
URDF_PATH="${SHARE_DIR}/urdf/robot_description.urdf"
SDF_PATH="$SHARE_DIR/urdf/swerve.sdf"

if [ ! -d $URDF_PATH ]; then
    touch $URDF_PATH
fi

echo
echo "Generating URDF"
xacro swerve_drive/urdf/swerve.urdf > $URDF_PATH
gz sdf -p $URDF_PATH > $SDF_PATH
echo "URDF written to" $URDF_PATH
echo "SDF written to" $SDF_PATH

source ~/.bashrc
