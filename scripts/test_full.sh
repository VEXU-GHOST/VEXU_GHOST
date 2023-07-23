#!/bin/bash

cd ~/VEXU_GHOST
colcon test --event-handlers console_direct+ --packages-skip Tutorial casadi matplotlib_cpp
