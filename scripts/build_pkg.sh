#!/bin/bash

PKG_LIST="$*"
# conda deactivate || true
# export Python3_EXECUTABLE=/usr/bin/python3
$VEXU_HOME/scripts/hardware/service.sh stop

echo ------ Building ------
colcon build --symlink-install --packages-up-to $PKG_LIST