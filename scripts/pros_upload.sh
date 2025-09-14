#!/bin/bash

bash $VEXU_HOME/scripts/pros_build.sh
cd $VEXU_HOME/02_V5/ghost_pros

echo
echo -------------------------------------------------------
echo ---------------- Uploading PROS Project ---------------
echo -------------------------------------------------------

pros upload