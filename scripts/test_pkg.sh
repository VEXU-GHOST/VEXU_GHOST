#!/bin/bash

PKG_LIST="$*"

echo ------ Building ------
colcon build --packages-up-to $PKG_LIST

RETURN=$?

if [ $RETURN -eq 0 ]; then
    echo ------ Testing ------
    colcon test --packages-select $PKG_LIST --event-handlers console_direct+
fi