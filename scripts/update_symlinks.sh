#!/bin/bash
V5_DIR="${HOME}/VEXU_GHOST/2_V5/ghost_pros"

##############
### V5 SRC ###
##############
cd $V5_DIR/src

# Clear existing directories
rm -r ghost_serial ghost_estimation ghost_control ghost_common
mkdir ghost_serial ghost_estimation ghost_control ghost_common

### Symlink ghost_serial ###
cd ghost_serial

# Add Folders
ln -s ../../../../1_Libraries/ghost_serial/src/cobs
ln -s ../../../../1_Libraries/ghost_serial/src/msg_parser
ln -s ../../../../1_Libraries/ghost_serial/src/serial_utils

# Add Specific Files
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../../1_Libraries/ghost_serial/src/base_interfaces/generic_serial_base.cpp
ln -s ../../../../../1_Libraries/ghost_serial/src/base_interfaces/v5_serial_base.cpp

### Symlink ghost_estimation ###
# Add Folders
cd ../../ghost_estimation
ln -s ../../../../1_Libraries/ghost_estimation/src/filters

### Symlink ghost_control ###
# Add Folders
cd ../ghost_control
ln -s ../../../../1_Libraries/ghost_control/src/motor_controller.cpp
mkdir models && cd models
ln -s ../../../../../1_Libraries/ghost_control/src/models/dc_motor_model.cpp

### Symlink ghost_common ###
# Add Folders
cd ../../ghost_common
ln -s ../../../../1_Libraries/ghost_common/src/v5_robot_config_defs.cpp
ln -s ../../../../1_Libraries/ghost_common/src/v5_robot_config.cpp

##################
### V5 INCLUDE ###
##################

cd $V5_DIR/include

# Clear existing directories
rm -r ghost_serial ghost_estimation ghost_control ghost_common
mkdir ghost_serial ghost_estimation ghost_control ghost_common

### Symlink ghost_serial ###
cd ghost_serial

# Add Folders
ln -s ../../../../1_Libraries/ghost_serial/include/ghost_serial/cobs
ln -s ../../../../1_Libraries/ghost_serial/include/ghost_serial/msg_parser
ln -s ../../../../1_Libraries/ghost_serial/include/ghost_serial/serial_utils

# Add Specific Files
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../../1_Libraries/ghost_serial/include/ghost_serial/base_interfaces/generic_serial_base.hpp
ln -s ../../../../../1_Libraries/ghost_serial/include/ghost_serial/base_interfaces/v5_serial_base.hpp

### Symlink ghost_estimation ###
# Add Folders
cd ../../ghost_estimation
ln -s ../../../../1_Libraries/ghost_estimation/include/ghost_estimation/filters

### Symlink ghost_control ###
# Add Folders
cd ../ghost_control
ln -s ../../../../1_Libraries/ghost_control/include/ghost_control/motor_controller.hpp
mkdir models && cd models
ln -s ../../../../../1_Libraries/ghost_control/include/ghost_control/models/dc_motor_model.hpp

### Symlink ghost_ros ###
# Add Folders
cd ../../ghost_common
ln -s ../../../../1_Libraries/ghost_common/include/ghost_common/v5_robot_config_defs.hpp
