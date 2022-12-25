#ifndef GHOST_ROS__SERIAL_INTERFACE_HPP
#define GHOST_ROS__SERIAL_INTERFACE_HPP

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <errno.h>
#include <mutex>

#include <iomanip>
#include <iostream>
#include <thread>
#include <atomic>

#include "msg_parser.hpp"


namespace ghost_serial {

class SerialInterface{
    public:
        SerialInterface(std::string port_name, std::string msg_start_seq, int incoming_msg_len);
        ~SerialInterface();
        
        bool trySerialOpen();
        bool writeMsgToSerial(unsigned char * buffer, int num_bytes);
        bool readMsgFromSerial(unsigned char * buffer);

    private:
        // Serial helpers
        void flushStream();
        int getNumBytesAvailable();

        // Config params
        std::string port_name_;
        std::string msg_start_seq_;
        int incoming_msg_len_;

        // Serial IO
        int serial_port_fd_;
        std::mutex serial_io_mutex_;
        std::atomic_bool port_open_;
        
        // Serial Reader
        struct pollfd pollfd_read_;
        std::unique_ptr<MsgParser> msg_parser_;
        std::vector<unsigned char> raw_serial_buffer_;
        std::vector<unsigned char> incoming_msg_;
};

} // namespace ghost_serial

#endif // GHOST_ROS__SERIAL_INTERFACE_HPP