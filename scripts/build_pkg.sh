#!/bin/bash

PKG_LIST="$*"

pkill ros

echo ------ Building ------
colcon build --symlink-install --packages-up-to $PKG_LIST