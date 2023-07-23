#!/bin/bash
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions ros-humble-xacro ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
sudo apt-get install -y ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rqt-tf-tree ros-humble-joy-linux
sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev
sudo apt-get install -y ros-humble-realsense2*
cd ~/VEXU_GHOST
git submodule update --init
