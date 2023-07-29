#!/bin/bash

bash $HOME/VEXU_GHOST/scripts/build_ros.sh

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