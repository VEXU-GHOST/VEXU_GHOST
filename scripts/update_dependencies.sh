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
echo "--------------- Sensor Host (RP2040) Firmware Toolchain ---------------"
# ARM cross toolchain + build tools for the Pico firmware. libusb/pkg-config
# are for building picotool below.
sudo apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib build-essential cmake libusb-1.0-0-dev pkg-config || exit -1

# Raspberry Pi Pico SDK (located by sensor_host.sh via PICO_SDK_PATH).
# Version pinned to match polling_firmware/CMakeLists.txt (sdkVersion 2.2.0).
PICO_SDK_DIR="$VEXU_HOME/09_External/pico-sdk"
if [ ! -d "$PICO_SDK_DIR/.git" ]; then
    git clone --branch 2.2.0 --depth 1 https://github.com/raspberrypi/pico-sdk.git "$PICO_SDK_DIR" || exit -1
fi
# Only tinyusb is needed (USB stdio); skip the Pico W wifi/bluetooth submodules.
git -C "$PICO_SDK_DIR" submodule update --init lib/tinyusb || exit -1

# picotool — not packaged on Ubuntu 22.04, so build it from source against the
# SDK. `cmake --install` also drops in udev rules so sensor_host.sh can flash
# without sudo / manual BOOTSEL.
PICOTOOL_DIR="$VEXU_HOME/09_External/picotool"
if [ ! -d "$PICOTOOL_DIR/.git" ]; then
    git clone --branch 2.2.0 --depth 1 https://github.com/raspberrypi/picotool.git "$PICOTOOL_DIR" || exit -1
fi
cmake -S "$PICOTOOL_DIR" -B "$PICOTOOL_DIR/build" -DPICO_SDK_PATH="$PICO_SDK_DIR" || exit -1
cmake --build "$PICOTOOL_DIR/build" -j"$(nproc)" || exit -1
sudo cmake --install "$PICOTOOL_DIR/build" || exit -1

echo
echo "--------------- ROSDEP Init ---------------"
sudo rosdep init

echo
echo "--------------- ROSDEP Update ---------------"
rosdep update || exit -1

echo
echo "--------------- ROSDEP Install ---------------"
rosdep install --from-paths . --ignore-src -r -y || exit -1

