#!/bin/bash

cd ~/VEXU
colcon build

URDF_PATH="$PWD/install/swerve_drive/share/swerve_drive/urdf/robot_description.urdf"

if [ ! -d $URDF_PATH ]; then
    touch $URDF_PATH
fi

echo
echo "Generating URDF"
xacro swerve_drive/urdf/swerve.urdf > $URDF_PATH
echo "URDF written to" $URDF_PATH

source ~/.bashrc