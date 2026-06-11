#!/bin/bash

# Help printout
if [ "$1" == "-h" ];
then
    echo "Builds the ROS workspace, then the V5 PROS project and the RP2040 sensor host firmware."
    exit 0
fi

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

$VEXU_HOME/scripts/ghost stop

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

cd $VEXU_HOME

# ---- Embedded targets (built/flashed after the ROS workspace) -------------

# V5 PROS project — requires the pros CLI and a connected V5 brain.
if command -v pros >/dev/null 2>&1
then
    echo
    bash scripts/pros_upload.sh
else
    echo
    echo "pros CLI not found; skipping V5 PROS build/upload."
fi

# RP2040 sensor host firmware — requires the Pico toolchain/SDK
# (scripts/update_dependencies.sh). Flashing is skipped if no board is attached.
if command -v arm-none-eabi-gcc >/dev/null 2>&1 && \
   { [ -n "$PICO_SDK_PATH" ] || [ -d "$VEXU_HOME/09_External/pico-sdk" ] || [ -d "$HOME/.pico-sdk/sdk" ]; }
then
    echo
    bash scripts/build/sensor_host.sh
else
    echo
    echo "Pico toolchain/SDK not found; skipping sensor host build."
    echo "Install it with scripts/update_dependencies.sh, then run scripts/build/sensor_host.sh"
fi
