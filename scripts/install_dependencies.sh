#!/bin/bash
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions ros-foxy-xacro ros-foxy-gazebo-ros ros-foxy-gazebo-ros-pkgs
sudo apt-get install -y ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-rqt-tf-tree ros-foxy-joy-linux
sudo apt-get install -y python3-pip

cd ~/
git clone git@github.com:jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make -j
sudo make install
