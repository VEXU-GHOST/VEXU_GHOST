#!/bin/bash

# Get processor architecture to determine if we should build simulator or not (not on robot hardware)
arch=$(uname -p)

# Assumes repository is in base directory
cd ~/VEXU_GHOST

# Build ignores simulator packages on embedded devices
if [ "$arch" == 'x86_64' ];
then 
    colcon build
fi

if [ "$arch" == 'aarch65' ];
then 
    colcon build --packages-select ghost_ros
fi

source install/setup.bash

# Process URDFs from Xacro and add to Share
GHOST_ROS_SHARE_DIR="$PWD/install/ghost_ros/share/ghost_ros"
URDF_PATH="${GHOST_ROS_SHARE_DIR}/urdf/ghost1.urdf"

if [ ! -d $GHOST_ROS_SHARE_DIR ];
then
    touch $URDF_PATH
else
    echo
    echo "Generating URDF"
    xacro ghost_ros/urdf/ghost1.xacro > $URDF_PATH
    echo "URDF written to" $URDF_PATH
fi

# Processes URDFs for simulation
if [ "$arch" == 'x86_64' ];
then 
    GHOST_SIM_SHARE_DIR="$PWD/install/ghost_sim/share/ghost_sim"
    URDF_SIM_VOLTAGE_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_voltage.urdf"
    URDF_SIM_PID_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_pid.urdf"

    if [ ! -d $GHOST_SIM_SHARE_DIR ];
    then
        touch $URDF_SIM_VOLTAGE_PATH
        touch $URDF_SIM_PID_PATH
    else
        echo
        echo "Generating URDF"
        xacro ghost_sim/urdf/ghost1_sim_voltage.xacro > $URDF_SIM_VOLTAGE_PATH
        xacro ghost_sim/urdf/ghost1_sim_pid.xacro > $URDF_SIM_PID_PATH
        echo "URDF written to" $URDF_SIM_VOLTAGE_PATH
        echo "URDF written to" $URDF_SIM_PID_PATH
    fi
fi

source ~/.bashrc
