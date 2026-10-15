#!/bin/bash
# Builds a PROS project. Defaults to the main robot project (02_V5/ghost_pros).
# Usage: pros_build.sh [project_dir]   e.g. pros_build.sh 02_V5/ghost_comms/manager

: "${VEXU_HOME:=$HOME/VEXU_GHOST}"
PROJECT_DIR="${1:-02_V5/ghost_pros}"
MAIN_PROJECT="02_V5/ghost_pros"

cd $VEXU_HOME
if [[ ! -d "$PROJECT_DIR" ]]; then
    echo "ERROR: No such project directory: $VEXU_HOME/$PROJECT_DIR"
    exit -1
fi

if [[ $(pros --version) ]] 2> /dev/null; then
    # Symlinks and the generated robot_config.hpp only apply to the main project;
    # the ghost_comms demos vendor their own sources.
    if [[ "$PROJECT_DIR" == "$MAIN_PROJECT" ]]; then
        echo
        echo -------------------------------------------------------
        echo ----------- Updating Project Symbolic Links -----------
        echo -------------------------------------------------------
        bash scripts/update_symlinks.sh || exit -1

        echo
        ./build/ghost_v5_interfaces/generate_pros_header robots.yaml || exit -1
    fi

    cd $PROJECT_DIR

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
    exit -1
fi
