#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <errno.h>

#include <iomanip>
#include <iostream>
#include <thread>
#include <atomic>

#include "yaml-cpp/yaml.h"

#include "COBS/COBS.hpp"
#include "serial/serial.h"
#include "globals/globals.hpp"

#ifndef GHOST_ROS__SERIAL_INTERFACE_HPP
#define GHOST_ROS__SERIAL_INTERFACE_HPP

namespace serial_interface {

class SerialInterface{
    public:
        SerialInterface(std::string config_file, std::function<void(unsigned char *)> read_callback_func);
        ~SerialInterface();
        
        void waitForSerialReady();
        void runSerialLoop(std::atomic_bool & run_flag);
        void writeMsg(unsigned char * msg);

    private:
        // Initialization methods
        void loadConfiguration(std::string filename);
        void writePipeInit();

        // Serial helpers
        int getAvailableBytesRead(int fd);
        void flushStream(int fd);

        // Read/Write Event Handlers
        void processSerialRead();
        void processSerialWrite();

        bool parseRawStream(const unsigned char * raw_msg_buffer, const int num_bytes, char * packet_decoded);

        YAML::Node config_yaml_;

        // Config params
        std::string port_name_;
        std::string msg_start_seq_;
        int max_bytes_to_read_;
        int max_msg_len_;
        std::chrono::duration<int, std::micro> max_msg_time_duration_;

        // Serial IO
        struct pollfd pollfd_read_write_[2];
        int serial_port_fd_;

        // Reader
        std::thread read_consumer_thread_;
        std::function<void(unsigned char[])> read_callback_func_;

        // Writer
        int write_pipe_input_fd_;
        int write_pipe_output_fd_;
        

        std::string output_string_test_;

};

} // namespace serial_interface

#endif