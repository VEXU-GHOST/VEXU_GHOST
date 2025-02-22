#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

cd $VEXU_HOME

echo
echo "--------------- Non-ROS Dependencies ---------------"
sudo apt-get install -y libgoogle-glog-dev cmake python3-colcon-common-extensions gfortran-10 libi2c-dev libi2c0 ccache || exit -1
sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev python3-rosdep2 apt-rdepends ros-humble-xacro sox libsox-fmt-mp3 || exit -1
pip install colcon-lint || exit -1
python3 -m pip install --upgrade pip
pip install setuptools==61 piper-tts==1.2.0

echo
echo "--------------- ROSDEP Init ---------------"
sudo rosdep init

echo
echo "--------------- ROSDEP Update ---------------"
rosdep update || exit -1

echo
echo "--------------- ROSDEP Install ---------------"
rosdep install --from-paths . --ignore-src -r -y || exit -1

