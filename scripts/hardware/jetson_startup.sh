#!/bin/bash

# put the path to this script in `gnome-session-properties` (likely /home/ghost2/VEXU_GHOST/scripts/hardware/jetson_startup.sh)

source ~/.bashrc

# https://askubuntu.com/questions/581922/export-display-value-in-shell-script
export DISPLAY=$(w -h $USER | awk '$2 ~ /:[0-9.]*/{print $2}')

logger "RUNNING THE gnome-terminal START LOOP NOW"

until gnome-terminal --wait --full-screen -- "$HOME/VEXU_GHOST/scripts/launch_hardware.sh"; do
	logger "RUNNING GNOME TERINMAL FAILED AND TRYING AGAIN IN 3 sec"
	sleep 3
done;

logger "DONE RUNNING THE LOOP"
