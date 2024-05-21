#!/bin/bash
cd ~/VEXU_GHOST
if [[ $(pros --version) ]] 2> /dev/null; then
    echo
    echo -------------------------------------------------------
	echo ----------- Updating Project Symbolic Links -----------
	echo -------------------------------------------------------
    bash scripts/update_symlinks.sh || exit -1
    
    echo 
    ./build/ghost_v5_interfaces/generate_pros_header robots.yaml || exit -1

    cd 02_V5/ghost_pros

    echo
    echo -------------------------------------------------------
    echo ---------------- Cleaning PROS Project ----------------
    echo -------------------------------------------------------
    make clean || exit -1

    echo 
    echo -------------------------------------------------------
    echo ---------------- Building PROS Project ----------------
    echo -------------------------------------------------------
    echo
    pros make || exit -1
else
    echo "ERROR: PROS is not installed!"
fi