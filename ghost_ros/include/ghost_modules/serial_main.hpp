#include <iostream>
#include "serial_interface/serial_interface.hpp"
#include "globals/globals.hpp"

namespace ghost_modules {
    
/**
 * @brief User defined callback to execute on new input serial msgs
 * 
 * @param msg raw bytes from serial port
 */
void serial_read_callback(unsigned char *msg);

/**
 * @brief Entry point for GHOST V5 Serial thread
 * 
 * @param config_file 
 */
void ghost_serial_main(std::string config_file);

} // ghost_modules