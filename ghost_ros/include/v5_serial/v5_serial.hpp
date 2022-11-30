#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <thread>

#include "yaml-cpp/yaml.h"

#include "COBS/COBS.hpp"
#include "serial/serial.h"
#include "globals/globals.hpp"

#ifndef GHOST_ROS__V5_SERIAL_HPP
#define GHOST_ROS__V5_SERIAL_HPP

namespace v5_serial {

void v5_serial_main(std::string config_file);

class V5SerialInterface{
    public:
        V5SerialInterface(std::string config_file);
        ~V5SerialInterface();
        
        void openPort();

        void startReaderThread();
        void startWriterThread();

        bool isConnected(){
            return connected_;
        }

    private:
        void loadConfiguration(std::string filename);
        void readerThreadLoop();
        void writerThreadLoop();

        YAML::Node config_yaml_;
        std::string port_name_;
        
        int serial_port_;
        bool connected_;

        std::thread reader_thread_;
        std::thread writer_thread_;
};

} // namespace v5_serial

#endif