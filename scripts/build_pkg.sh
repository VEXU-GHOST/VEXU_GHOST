#!/bin/bash

PKG_LIST="$*"

echo ------ Building ------
colcon build --symlink-install --packages-up-to $PKG_LIST