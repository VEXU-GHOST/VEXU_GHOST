#!/bin/bash

if [ "$1" == "-h" ];
then
    echo "Builds ROS and PROS Workspace."
    echo "Specify -r to skip PROS build"
    exit 0
fi

# Get processor architecture to determine if we should build simulator or not (not on robot hardware)
arch=$(uname -p)

# Assumes repository is in base directory
cd ~/VEXU_GHOST
echo "---Building Ghost ROS Packages---"

# Build ignores simulator packages on embedded devices
if [ "$arch" == 'x86_64' ];
then 
    colcon build --packages-skip casadi matplotlib_cpp Tutorial --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

if [ "$arch" == 'aarch64' ];
then 
    colcon build --packages-up-to ghost_ros --packages-skip casadi matplotlib_cpp  Tutorial --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

source install/setup.bash

# Process URDFs from Xacro and add to Share
GHOST_DESCRIPTION_SHARE_DIR="$PWD/install/ghost_description/share/ghost_description"
URDF_PATH="${GHOST_DESCRIPTION_SHARE_DIR}/urdf/ghost1.urdf"

if [ ! -d $GHOST_DESCRIPTION_SHARE_DIR ];
then
    touch $URDF_PATH
else
    echo
    echo "---Generating Ghost Description URDF---"
    xacro ghost_description/urdf/ghost1.xacro > $URDF_PATH
    echo "URDF written to" $URDF_PATH
fi

# Processes URDFs for simulation
if [ "$arch" == 'x86_64' ];
then 
    GHOST_SIM_SHARE_DIR="$PWD/install/ghost_sim/share/ghost_sim"
    URDF_SIM_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_base.urdf"

    if [ ! -d $GHOST_SIM_SHARE_DIR ];
    then
        touch $URDF_SIM_PATH
    else
        echo
        echo "---Generating Ghost Simulation URDFs---"
        xacro ghost_sim/urdf/ghost1_sim_base.xacro > $URDF_SIM_PATH
        echo "URDF written to" $URDF_SIM_PATH
    fi
fi

source ~/.bashrc

if [ "$1" != "-r" ];
then
    echo 
    echo "Checking for PROS"
    if [[ $(pros --version) ]] 2> /dev/null; then
        echo "Found PROS"
        echo "---Updating V5 Project Symbolic Links---"
        bash scripts/update_symlinks.sh

        cd ghost_pros

        echo
        echo "---Cleaning PROS Project---"
        make clean

        echo 
        echo "---Building PROS Project---"
        pros make
    else
        echo "PROS not installed, skipping V5 Build"
    fi
fi