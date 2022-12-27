#ifndef GHOST_ROS__SERIAL_INTERFACE_HPP
#define GHOST_ROS__SERIAL_INTERFACE_HPP

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
#include <thread>
#include <atomic>

#include "../msg_parser/msg_parser.hpp"

namespace ghost_serial
{

    class SerialInterface
    {
    public:
        SerialInterface(
            std::string port_name,
            std::string msg_start_seq,
            int msg_len,
            bool use_checksum = false,
            bool verbose = false);

        ~SerialInterface();
        /**
         * @brief Attempts once to open serial port, catching errors and printing to stdout.
         *
         * @return bool if port was opened or not
         */
        bool trySerialOpen();

        /**
         * @brief Thread-safe method to write msg buffer to serial port. Shares mutex with reader thread,
         * so may block for duration of a serial read (very short period).
         *
         * Applies COBS Encoding to msg before writing to serial.
         * Calculates and appends msg checksum if configured.
         *
         * @param buffer    msg to write to serial
         * @param num_bytes length of msg in bytes
         * @return bool if write was successful
         */
        bool writeMsgToSerial(const unsigned char *buffer, const int num_bytes);

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
        bool readMsgFromSerial(unsigned char *msg_buffer);

    private:
        /**
         * @brief Flushes buffer from serial input stream (reader), clearing old data
         */
        void flushStream() const;

        /**
         * @brief Returns bytes available to read from serial port file descriptor
         * 
         * @return int num_bytes_available
         */
        int getNumBytesAvailable() const;

        /**
         * @brief Sums all bytes in msg as 8-bit integers to generate checksum for msg validation.
         * 
         * @param buffer    msg buffer
         * @param num_bytes length of msg in bytes
         * @return uint8_t checksum
         */
        uint8_t calculateChecksum(const unsigned char buffer[], const int &num_bytes) const;

        /**
         * @brief Sets termios flags for serial port settings. Follows this nearly verbatim:
         * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
         * 
         * @return bool if tty port config was set without error
         */
        bool setSerialPortConfig();

        // Config params
        std::string port_name_;
        std::string msg_start_seq_;
        int msg_len_;
        bool use_checksum_;
        bool verbose_;

        // Serial IO
        int serial_port_fd_;
        struct pollfd pollfd_read_;

        std::mutex serial_io_mutex_;
        std::atomic_bool port_open_;

        std::vector<unsigned char> raw_serial_buffer_;
        std::unique_ptr<MsgParser> msg_parser_;
    };

} // namespace ghost_serial

#endif // GHOST_ROS__SERIAL_INTERFACE_HPP