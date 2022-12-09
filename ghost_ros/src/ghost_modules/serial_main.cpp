#include "ghost_modules/serial_main.hpp"

namespace ghost_modules {

void serial_read_callback(unsigned char *msg)
{
    std::cout << "Read Callback" << std::endl;
}

void ghost_serial_main(std::string config_file)
{

    serial_interface::SerialInterface ghost_serial(config_file, serial_read_callback);

    // Try to open serial port (retrying until success)
    ghost_serial.waitForSerialReady();

    // Run serial loop until program exits
    ghost_serial.runSerialLoop(globals::run);
}

} // ghost_modules