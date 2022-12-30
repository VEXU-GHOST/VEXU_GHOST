#ifndef GHOST_ROS__JETSON_SERIAL_INTERFACE_HPP
#define GHOST_ROS__JETSON_SERIAL_INTERFACE_HPP

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <errno.h>
#include <termios.h>
#include <mutex>

#include <iomanip>
#include <iostream>

#include "../msg_parser/msg_parser.hpp"
#include "ghost_serial/serial_interface/base_serial_interface.hpp"

namespace ghost_serial
{

    class JetsonSerialInterface : public BaseSerialInterface
    {
    public:
        JetsonSerialInterface(
            std::string port_name,
            std::string msg_start_seq,
            int msg_len,
            bool use_checksum = false);

        ~JetsonSerialInterface();
        
        /**
         * @brief Attempts once to open serial port, catching errors and printing to stdout.
         *
         * @return bool if port was opened or not
         */
        bool trySerialInit();

        /**
         * @brief Thread-safe method to read serial port for new msgs. Blocks until new msg is recieved
         * or 1s timeout elapses. Implementation uses Poll() system call to avoid busy-waiting. Highly efficient!
         *
         * Internally applies COBS decoding and checksum for msg validation. Searches for specified start sequence
         * and then reads for msg_len until null delimiter is found.
         *
         * @param msg_buffer buffer of length msg_len to store incoming serial msgs
         * @return bool if msg was found in serial stream
         */
        bool readMsgFromSerial(unsigned char msg_buffer[]) override;

    private:
        /**
         * @brief Flushes buffer from serial input stream (reader), clearing old data.
         
         * @return false if flush fails
         */
        bool flushStream() const override;

        /**
         * @brief Returns bytes available to read from serial port file descriptor
         * 
         * @return int num_bytes_available
         */
        int getNumBytesAvailable() const override;

        /**
         * @brief Sets termios flags for serial port settings. Follows this nearly verbatim:
         * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
         * 
         * @return bool if tty port config was set without error
         */
        bool setSerialPortConfig() override;

        // Config params
        std::string port_name_;

        // Serial IO
        struct pollfd pollfd_read_;
    };

} // namespace ghost_serial

#endif // GHOST_ROS__JETSON_SERIAL_INTERFACE_HPP