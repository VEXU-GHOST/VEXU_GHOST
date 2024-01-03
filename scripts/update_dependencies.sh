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
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions gfortran-10

# Exit if step failed
if [[ $? -ne 0 ]]; then
    exit -1
fi

sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev python3-rosdep2 apt-rdepends

# Exit if step failed
if [[ $? -ne 0 ]]; then
    exit -1
fi

pip install colcon-lint


# Exit if step failed
if [[ $? -ne 0 ]]; then
    exit -1
fi


echo
echo "--------------- ROSDEP Init ---------------"
sudo rosdep init

echo
echo "--------------- ROSDEP Update ---------------"
rosdep update

# Exit if step failed
if [[ $? -ne 0 ]]; then
    exit -1
fi

echo
echo "--------------- ROSDEP Install ---------------"
rosdep install --from-paths . --ignore-src -r -y

# Exit if step failed
if [[ $? -ne 0 ]]; then
    exit -1
fi

exit $exit_code