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

$VEXU_HOME/scripts/hardware/service.sh stop

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

sim_pkgs=(
    ghost_sim
    ghost_sim_examples
    ghost_viz
    plotjuggler
    plotjuggler_ros
)

docker_skip=(
    ghost_io
    ghost_ros_interfaces
    ghost_motion_planner_core
    ghost_sim
    ghost_example_robot
    ghost_swerve_mpc_planner
    ghost_sensing
    ghost_tank
    ghost_sim_examples
    ghost_high_stakes
)

# Ignore simulator packages on embedded devices
if [[ $(uname -p) == "aarch64" ]]; then
    skip=( "${skip[@]}" "${sim_pkgs[@]}")
fi

# Ignore docker-incompatible packages
if [[ -f /.dockerenv ]]; then
    skip=( "${skip[@]}" "${docker_skip[@]}")
fi

echo "${skip[@]}"

colcon build --symlink-install --packages-skip ${skip[@]} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit -1

if ! command -v pros 2>&1 >/dev/null
then
    exit;
fi

$VEXU_HOME/scripts/pros_upload.sh