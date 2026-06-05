#!/bin/bash

# Help printout
if [ "$1" == "-h" ];
then
    echo "Builds ROS and PROS Workspace."
    echo "Pass package names to build only those (and their dependencies)."
    echo "Specify -r to skip PROS build"
    exit 0
fi

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

$VEXU_HOME/scripts/ghost.sh stop

# Assumes repository is in base directory
cd $VEXU_HOME

# If package names are given, build only those (and their dependencies) and exit.
if [ "$#" -gt 0 ];
then
    echo "Building $* packages..."
    colcon build --symlink-install --packages-up-to "$@" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit -1
    exit 0
fi

echo "Building all packages... give package names to build specific ones."

# Get processor architecture to determine if we should build simulator or not (not on robot hardware)
arch=$(uname -p)

echo "---Building Ghost ROS Packages---"

skip=(
    behaviortree_cpp
    behaviortree_ros2
    btcpp_ros2_interfaces
    btcpp_ros2_samples
    rplidar_ros
    ghost_swerve
    plotjuggler
    plotjuggler_ros
    librealsense2
)

# Build ignores simulator packages on embedded devices
if [ "$arch" == 'x86_64' ];
then 
    colcon build --symlink-install --packages-skip ${skip[@]} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit -1
fi

if [ "$arch" == 'aarch64' ];
then 
    colcon build --symlink-install --packages-skip ${skip[@]} ghost_sim ghost_sim_examples ghost_viz plotjuggler plotjuggler_ros --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit -1
fi

if ! command -v pros 2>&1 >/dev/null
then
    exit;
fi

cd $VEXU_HOME
echo
bash scripts/build/pros_upload.sh