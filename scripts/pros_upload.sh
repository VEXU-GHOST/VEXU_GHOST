#!/bin/bash
# Builds and uploads a PROS project. Defaults to the main robot project (02_V5/ghost_pros).
# Usage: pros_upload.sh [project_dir]   e.g. pros_upload.sh 02_V5/ghost_comms/worker

: "${VEXU_HOME:=$HOME/VEXU_GHOST}"
PROJECT_DIR="${1:-02_V5/ghost_pros}"

bash $VEXU_HOME/scripts/pros_build.sh "$PROJECT_DIR" || exit -1
cd $VEXU_HOME/$PROJECT_DIR

echo
echo -------------------------------------------------------
echo ---------------- Uploading PROS Project ---------------
echo -------------------------------------------------------

pros upload
