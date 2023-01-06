#!/bin/bash
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions ros-foxy-xacro ros-foxy-gazebo-ros ros-foxy-gazebo-ros-pkgs
sudo apt-get install -y ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-rqt-tf-tree ros-foxy-joy-linux
sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev
sudo apt-get install -y ros-foxy-rplidar-ros  ros-foxy-realsense2*

echo "Building yaml-cpp from source in ~/ghost_deps"
cd ~/
mkdir ghost_deps && cd ghost_deps
git clone git@github.com:jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make -j
sudo make install

cd ~/VEXU_GHOST
git submodule update --init

echo "Adding /usr/local/lib to LD_LIBRARY_PATH in ~/.bashrc"
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc