#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

if [ ! -f "${VEXU_HOME}/install/setup.bash" ]
then
    echo "Failure: workspace is not built. Run ./scripts/build.sh before lint."
    exit -1
fi

cd "${VEXU_HOME}"

source /opt/ros/humble/setup.bash
source "${VEXU_HOME}/install/setup.bash"

skip=(
    ghost_swerve
    plotjuggler
    plotjuggler_ros
    ghost_viz
    ghost_localization
    ghost_sim
)

echo "---Running colcon lint---"
colcon lint --packages-skip "${skip[@]}" || exit -1
