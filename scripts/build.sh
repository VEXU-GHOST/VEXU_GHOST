#!/bin/bash

# Assumes repository is in base directory
cd ~/VEXU_GHOST
echo "---Building Ghost ROS Packages---"

# Build simulator packages depending on what is passed for EMBEDBUILD
if [ "$1" == "EMBEDBUILD" ];
then 
    colcon build --symlink-install --packages-skip ghost_sim
else
    colcon build --symlink-install
fi

source install/setup.bash

bash scripts/generate_urdfs.sh

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