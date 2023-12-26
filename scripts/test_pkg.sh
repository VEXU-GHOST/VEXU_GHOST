#!/bin/bash

echo ------ Building ------
colcon build --packages-up-to $1

RETURN=$?

if [ $RETURN -eq 0 ]; then
    echo ------ Testing ------
    colcon test --packages-select $1 --event-handlers console_direct+
fi