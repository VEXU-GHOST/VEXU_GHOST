#!/bin/bash
cd ~/VEXU_GHOST

echo
echo "--------------- Non-ROS Dependencies ---------------"
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions gfortran-10
sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev python3-rosdep2 apt-rdepends
pip install colcon-lint

echo
echo "--------------- ROSDEP Init ---------------"
sudo rosdep init

echo
echo "--------------- ROSDEP Update ---------------"
rosdep update

echo
echo "--------------- ROSDEP Install ---------------"
rosdep install --from-paths . --ignore-src -r -y
