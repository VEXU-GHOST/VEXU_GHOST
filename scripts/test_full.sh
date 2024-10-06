#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

skip=(
    behaviortree_cpp
    behaviortree_ros2
    btcpp_ros2_interfaces
    btcpp_ros2_samples
    rplidar_ros
)

cd $VEXU_HOME
colcon test --packages-skip ${skip[@]} --return-code-on-test-failure --event-handlers console_direct+ || exit -1
