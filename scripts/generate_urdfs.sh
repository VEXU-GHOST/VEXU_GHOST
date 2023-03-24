#!/bin/bash

cd ~/VEXU_GHOST

# Process URDFs from Xacro and add to Share
GHOST_DESCRIPTION_SHARE_DIR="$PWD/install/ghost_description/share/ghost_description"
URDF_PATH="${GHOST_DESCRIPTION_SHARE_DIR}/urdf_robomasters/sentry.urdf"

if [ ! -d $GHOST_DESCRIPTION_SHARE_DIR ];
then
    touch $URDF_PATH
else
    echo
    echo "---Generating Ghost Description URDF---"
    xacro ghost_description/urdf_robomasters/sentry.xacro > $URDF_PATH
    echo "URDF written to" $URDF_PATH
fi

# Processes URDFs for simulation
if [ "$1" != "EMBEDBUILD" ];
then 
    GHOST_SIM_SHARE_DIR="$PWD/install/ghost_sim/share/ghost_sim"
    URDF_SIM_BASE="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_base.urdf"
    URDF_SIM_VOLTAGE_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_voltage.urdf"
    URDF_SIM_PID_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_pid.urdf"
    URDF_SIM_PATH="${GHOST_SIM_SHARE_DIR}/urdf_robomasters/sentry_base.urdf"

    if [ ! -d $GHOST_SIM_SHARE_DIR ];
    then
        touch $URDF_SIM_BASE
        touch $URDF_SIM_VOLTAGE_PATH
        touch $URDF_SIM_PID_PATH
        touch $URDF_SIM_PATH
    else
        echo
        echo "---Generating Ghost Simulation URDFs---"
        xacro ghost_sim/urdf/ghost1_sim_base.xacro > $URDF_SIM_BASE
        xacro ghost_sim/urdf/ghost1_sim_voltage.xacro > $URDF_SIM_VOLTAGE_PATH
        xacro ghost_sim/urdf/ghost1_sim_pid.xacro > $URDF_SIM_PID_PATH
        xacro ghost_sim/urdf_robomasters/sentry_base.xacro > $URDF_SIM_PATH
        echo "URDF written to" $URDF_SIM_BASE 
        echo "URDF written to" $URDF_SIM_VOLTAGE_PATH
        echo "URDF written to" $URDF_SIM_PID_PATH
        echo "URDF written to" $URDF_SIM_PATH
    fi
fi

source ~/.bashrc