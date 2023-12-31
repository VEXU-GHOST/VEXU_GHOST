#!/bin/bash
cd ~/VEXU_GHOST
if [[ $(pros --version) ]] 2> /dev/null; then
    echo
    echo -------------------------------------------------------
	echo ----------- Updating Project Symbolic Links -----------
	echo -------------------------------------------------------
    bash scripts/update_symlinks.sh
    
    echo 
    ./build/ghost_v5_interfaces/generate_pros_header robots.yaml

    RETURN=$?
    if [ $RETURN -eq 0 ]; then
        cd 02_V5/ghost_pros

        echo
        echo -------------------------------------------------------
        echo ---------------- Cleaning PROS Project ----------------
        echo -------------------------------------------------------
        make clean

        echo 
        echo -------------------------------------------------------
        echo ---------------- Building PROS Project ----------------
        echo -------------------------------------------------------
        echo
        pros make
    fi
else
    echo "ERROR: PROS is not installed!"
fi