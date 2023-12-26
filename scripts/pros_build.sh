#!/bin/bash
cd ~/VEXU_GHOST
echo "Checking for PROS"
if [[ $(pros --version) ]] 2> /dev/null; then
    echo "Found PROS"
    echo "---Updating V5 Project Symbolic Links---"
    bash scripts/update_symlinks.sh
    
    echo 
    ./build/ghost_v5_interfaces/generate_pros_header robots.yaml

    RETURN=$?
    if [ $RETURN -eq 0 ]; then
        cd 02_V5/ghost_pros

        echo
        echo "---Cleaning PROS Project---"
        make clean

        echo 
        echo "---Building PROS Project---"
        pros make
    fi
else
    echo "PROS not installed"
fi