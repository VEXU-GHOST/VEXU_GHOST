#!/bin/bash

echo ------ Building ------
colcon build --packages-select $1

echo

echo ------ Testing ------

colcon test --packages-select $1 --event-handlers console_direct+