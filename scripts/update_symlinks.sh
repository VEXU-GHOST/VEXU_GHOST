#!/bin/bash
V5_DIR="${HOME}/VEXU_GHOST/02_V5/ghost_pros"

##############
### V5 SRC ###
##############
cd $V5_DIR/src

# Clear existing directories
rm -r ghost_serial ghost_v5_core ghost_util
mkdir ghost_serial ghost_v5_core ghost_util

### Symlink ghost_serial ###
cd ghost_serial

# Add Folders
ln -s ../../../../01_Libraries/ghost_serial/src/cobs
ln -s ../../../../01_Libraries/ghost_serial/src/msg_parser

# Add Specific Files
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../../01_Libraries/ghost_serial/src/base_interfaces/generic_serial_base.cpp
ln -s ../../../../../01_Libraries/ghost_serial/src/base_interfaces/v5_serial_base.cpp

### Symlink ghost_v5_core ###
# Add Folders
cd ../../ghost_v5_core
ln -s ../../../../01_Libraries/ghost_v5_core/src/filters
ln -s ../../../../01_Libraries/ghost_v5_core/src/motor

cd ../ghost_util
ln -s ../../../../01_Libraries/ghost_util/src/angle_util.cpp
ln -s ../../../../01_Libraries/ghost_util/src/bitmasks.cpp

##################
### V5 INCLUDE ###
##################

cd $V5_DIR/include

# Clear existing directories
rm -r ghost_serial ghost_v5_core ghost_util
mkdir ghost_serial ghost_v5_core ghost_util

### Symlink ghost_serial ###
cd ghost_serial

# Add Folders
ln -s ../../../../01_Libraries/ghost_serial/include/ghost_serial/cobs
ln -s ../../../../01_Libraries/ghost_serial/include/ghost_serial/msg_parser

# Add Specific Files
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../../01_Libraries/ghost_serial/include/ghost_serial/base_interfaces/generic_serial_base.hpp
ln -s ../../../../../01_Libraries/ghost_serial/include/ghost_serial/base_interfaces/v5_serial_base.hpp

### Symlink ghost_v5_core ###
# Add Folders
cd ../../ghost_v5_core
ln -s ../../../../01_Libraries/ghost_v5_core/include/ghost_v5_core/filters
ln -s ../../../../01_Libraries/ghost_v5_core/include/ghost_v5_core/motor
mkdir devices && cd devices
ln -s ../../../../../01_Libraries/ghost_v5_core/include/ghost_v5_core/devices/base
mkdir motor && cd motor
ln -s ../../../../../../01_Libraries/ghost_v5_core/include/ghost_v5_core/devices/motor/motor_device_interface.hpp
cd ..
mkdir rotation_sensor && cd rotation_sensor
ln -s ../../../../../../01_Libraries/ghost_v5_core/include/ghost_v5_core/devices/rotation_sensor/rotation_sensor_device_interface.hpp

cd ../../../ghost_util
ln -s ../../../../01_Libraries/ghost_util/include/ghost_util/bitmasks.hpp
ln -s ../../../../01_Libraries/ghost_util/include/ghost_util/angle_util.hpp