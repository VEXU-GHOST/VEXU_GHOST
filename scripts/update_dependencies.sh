#!/bin/bash
cd ~/VEXU_GHOST

echo
echo "--------------- ROSDEP Init ---------------"
sudo rosdep init

echo
echo "--------------- ROSDEP Update ---------------"
rosdep update

echo
echo "--------------- ROSDEP Install ---------------"
rosdep install --from-paths . --ignore-src -r -y
