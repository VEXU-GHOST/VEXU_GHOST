#!/bin/bash

PKG_LIST="$*"

$VEXU_HOME/scripts/hardware/service.sh stop

echo ------ Building ------
colcon build --symlink-install --packages-up-to $PKG_LIST