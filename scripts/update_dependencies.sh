#!/bin/bash

exit_code=0

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

# Update exit code if step failed
if [[ $? -ne 0 ]]; then
    exit_code=-1
fi

sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev python3-rosdep2 apt-rdepends

# Update exit code if step failed
if [[ $? -ne 0 ]]; then
    exit_code=-1
fi

pip install colcon-lint


# Update exit code if step failed
if [[ $? -ne 0 ]]; then
    exit_code=-1
fi


echo
echo "--------------- ROSDEP Init ---------------"
sudo rosdep init

# Update exit code if step failed
if [[ $? -ne 0 ]]; then
    exit_code=-1
fi

echo
echo "--------------- ROSDEP Update ---------------"
rosdep update

# Update exit code if step failed
if [[ $? -ne 0 ]]; then
    exit_code=-1
fi

echo
echo "--------------- ROSDEP Install ---------------"
rosdep install --from-paths . --ignore-src -r -y

# Update exit code if step failed
if [[ $? -ne 0 ]]; then
    exit_code=-1
fi

exit $exit_code