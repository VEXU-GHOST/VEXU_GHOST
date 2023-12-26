#!/bin/bash
V5_DIR="${HOME}/VEXU_GHOST/02_V5/ghost_pros"

##############
### V5 SRC ###
##############
cd $V5_DIR/src

# Clear existing directories
rm -r ghost_serial ghost_util ghost_estimation ghost_control ghost_v5_interfaces
mkdir ghost_serial ghost_util ghost_estimation ghost_control ghost_v5_interfaces

### Symlink ghost_serial ###
cd ghost_serial
ln -s ../../../../01_Libraries/ghost_serial/src/cobs
ln -s ../../../../01_Libraries/ghost_serial/src/msg_parser
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../../01_Libraries/ghost_serial/src/base_interfaces/generic_serial_base.cpp
ln -s ../../../../../01_Libraries/ghost_serial/src/base_interfaces/v5_serial_base.cpp

### Symlink ghost_util ###
cd $V5_DIR/src/ghost_util
ln -s ../../../../01_Libraries/ghost_util/src/angle_util.cpp
ln -s ../../../../01_Libraries/ghost_util/src/byte_utils.cpp

### Symlink ghost_estimation ###
cd $V5_DIR/src/ghost_estimation
ln -s ../../../../01_Libraries/ghost_estimation/src/filters

### Symlink ghost_control ###
cd $V5_DIR/src/ghost_control
ln -s ../../../../01_Libraries/ghost_control/src/motor_controller.cpp
mkdir models && cd models
ln -s ../../../../../01_Libraries/ghost_control/src/models/dc_motor_model.cpp

cd $V5_DIR/src/ghost_v5_interfaces
ln -s ../../../../01_Libraries/ghost_v5_interfaces/src/robot_hardware_interface.cpp

##################
### V5 INCLUDE ###
##################
cd $V5_DIR/include

# Clear existing directories
rm -r ghost_serial ghost_v5_interfaces ghost_util ghost_estimation ghost_control
mkdir ghost_serial ghost_v5_interfaces ghost_util ghost_estimation ghost_control

### Symlink ghost_serial ###
cd ghost_serial
ln -s ../../../../01_Libraries/ghost_serial/include/ghost_serial/cobs
ln -s ../../../../01_Libraries/ghost_serial/include/ghost_serial/msg_parser
mkdir base_interfaces && cd base_interfaces
ln -s ../../../../../01_Libraries/ghost_serial/include/ghost_serial/base_interfaces/generic_serial_base.hpp
ln -s ../../../../../01_Libraries/ghost_serial/include/ghost_serial/base_interfaces/v5_serial_base.hpp

### Symlink ghost_v5_interfaces ###
cd $V5_DIR/include/ghost_v5_interfaces
ln -s ../../../../01_Libraries/ghost_v5_interfaces/include/ghost_v5_interfaces/devices
ln -s ../../../../01_Libraries/ghost_v5_interfaces/include/ghost_v5_interfaces/robot_hardware_interface.hpp

### Symlink ghost_util ###
cd $V5_DIR/include/ghost_util
ln -s ../../../../01_Libraries/ghost_util/include/ghost_util/byte_utils.hpp
ln -s ../../../../01_Libraries/ghost_util/include/ghost_util/angle_util.hpp

### Symlink ghost_estimation ###
cd $V5_DIR/include/ghost_estimation
ln -s ../../../../01_Libraries/ghost_estimation/include/ghost_estimation/filters

### Symlink ghost_control ###
cd $V5_DIR/include/ghost_control
ln -s ../../../../01_Libraries/ghost_control/include/ghost_control/motor_controller.hpp
mkdir models && cd models
ln -s ../../../../../01_Libraries/ghost_control/include/ghost_control/models/dc_motor_model.hpp