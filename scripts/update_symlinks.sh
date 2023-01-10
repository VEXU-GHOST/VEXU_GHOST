#!/bin/bash
V5_DIR="$PWD/ghost_pros"

##############
### V5 SRC ###
##############
cd $V5_DIR/src

# Clear existing directories
rm -r ghost_serial ghost_estimation ghost_control ghost_ros
mkdir ghost_serial ghost_estimation ghost_control ghost_ros

### Symlink ghost_serial ###
cd ghost_serial

# Add Folders
ln -s ../../../ghost_serial/src/cobs
ln -s ../../../ghost_serial/src/msg_parser
ln -s ../../../ghost_serial/src/serial_utils

# Add Specific Files
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../ghost_serial/src/base_interfaces/generic_serial_base.cpp
ln -s ../../../../ghost_serial/src/base_interfaces/v5_serial_base.cpp

### Symlink ghost_estimation ###
# Add Folders
cd ../../ghost_estimation
ln -s ../../../ghost_estimation/src/filters

### Symlink ghost_control ###
# Add Folders
cd ../ghost_control
ln -s ../../../ghost_control/src/models

### Symlink ghost_ros ###
# Add Folders
cd ../ghost_ros
ln -s ../../../ghost_ros/src/robot_config

##################
### V5 INCLUDE ###
##################

cd $V5_DIR/include

# Clear existing directories
rm -r ghost_serial ghost_estimation ghost_control ghost_ros
mkdir ghost_serial ghost_estimation ghost_control ghost_ros

### Symlink ghost_serial ###
cd ghost_serial

# Add Folders
ln -s ../../../ghost_serial/include/ghost_serial/cobs
ln -s ../../../ghost_serial/include/ghost_serial/msg_parser
ln -s ../../../ghost_serial/include/ghost_serial/serial_utils

# Add Specific Files
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../ghost_serial/include/ghost_serial/base_interfaces/generic_serial_base.hpp
ln -s ../../../../ghost_serial/include/ghost_serial/base_interfaces/v5_serial_base.hpp

### Symlink ghost_estimation ###
# Add Folders
cd ../../ghost_estimation
ln -s ../../../ghost_estimation/include/ghost_estimation/filters

### Symlink ghost_control ###
# Add Folders
cd ../ghost_control
ln -s ../../../ghost_control/include/ghost_control/models

### Symlink ghost_ros ###
# Add Folders
cd ../ghost_ros
ln -s ../../../ghost_ros/include/ghost_ros/robot_config
