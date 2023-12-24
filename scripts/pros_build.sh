#!/bin/bash
cd ~/VEXU_GHOST
echo "Checking for PROS"
if [[ $(pros --version) ]] 2> /dev/null; then
    echo "Found PROS"
    echo "---Updating V5 Project Symbolic Links---"
    bash scripts/update_symlinks.sh

    cd 02_V5/ghost_pros

    echo
    echo "---Cleaning PROS Project---"
    make clean

    echo 
    echo "---Building PROS Project---"
    pros make
else
    echo "PROS not installed"
fi