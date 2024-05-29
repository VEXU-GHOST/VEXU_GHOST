#!/bin/bash

# Help printout
if [ "$1" == "-h" ];
then
    echo "Builds ROS and PROS Workspace."
    echo "Specify -r to skip PROS build"
    exit 0
fi

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

pkill ros -f

# Get processor architecture to determine if we should build simulator or not (not on robot hardware)
arch=$(uname -p)

# Assumes repository is in base directory
cd $VEXU_HOME
echo "---Building Ghost ROS Packages---"

skip=(
    behaviortree_cpp
    behaviortree_ros2
    btcpp_ros2_interfaces
    btcpp_ros2_samples
    rplidar_ros
)

# Build ignores simulator packages on embedded devices
if [ "$arch" == 'x86_64' ];
then 
    colcon build --symlink-install --packages-skip ${skip[@]} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit -1
fi

if [ "$arch" == 'aarch64' ];
then 
    colcon build --symlink-install --packages-skip ${skip[@]} ghost_sim ghost_viz plotjuggler plotjuggler-ros-plugins --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit -1
fi

if [ "$1" != "-r" ];
then
  cd $VEXU_HOME
  echo
  bash scripts/pros_upload.sh
fi