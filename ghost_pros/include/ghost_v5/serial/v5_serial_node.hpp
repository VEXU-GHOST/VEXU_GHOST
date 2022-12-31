#ifndef GHOST_V5__V5_SERIAL_NODE_HPP
#define GHOST_V5__V5_SERIAL_NODE_HPP

#include <atomic>
#include <memory>

#include "pros/apix.h"

#include "ghost_serial/base_interfaces/v5_serial_base.hpp"

namespace ghost_v5{
    class V5SerialNode
    {
    public:
        V5SerialNode(std::string msg_start_seq, int msg_len, bool use_checksum);
        ~V5SerialNode();

        void initSerial();

    private:
        // Background thread loop for processing serial reads
        void readerLoop();

        void updateActuatorCommands(unsigned char buffer[]);
        void writeSensorUpdate();

        // Config Params
        int msg_len_;
        bool using_reader_thread_;

        // Serial Interface
        std::unique_ptr<ghost_serial::V5SerialBase> serial_base_interface_;
        std::vector<unsigned char> new_msg_;

        // Reader Thread
        std::unique_ptr<pros::Task> reader_thread_;
        std::atomic_bool reader_thread_init_;
    };
} // namespace ghost_v5
#endif // GHOST_V5__V5_SERIAL_NODE_HPP