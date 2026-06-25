#!/bin/bash

# Help printout
if [ "$1" == "-h" ];
then
    echo "Builds ROS and PROS Workspace."
    echo "  -r  Skip PROS (V5 firmware) build and upload."
    echo "  In Docker, PROS is skipped unless VEXU_BUILD_PROS=1 (needs arm-none-eabi-g++ and a V5 for upload)."
    exit 0
fi

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

$VEXU_HOME/scripts/hardware/service.sh stop

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

# PROS needs the ARM toolchain and usually a USB V5; the image has pros-cli but not gcc-arm-none-eabi.
skip_pros=0
if [ "$1" == "-r" ]; then
	skip_pros=1
fi
if [ -n "${VEXU_IN_DOCKER}" ] && [ -z "${VEXU_BUILD_PROS:-}" ]; then
	skip_pros=1
	echo "--- Skipping PROS in Docker (use native Linux for full PROS, or set VEXU_BUILD_PROS=1 after installing gcc-arm-none-eabi) ---"
fi
if [ "$skip_pros" == "1" ]; then
	exit 0
fi

if ! command -v pros 2>&1 >/dev/null
then
    exit;
fi

cd $VEXU_HOME
echo
bash scripts/pros_upload.sh