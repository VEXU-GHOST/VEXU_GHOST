#ifndef GHOST_V5__V5_SERIAL_NODE_HPP
#define GHOST_V5__V5_SERIAL_NODE_HPP

#include <atomic>
#include <memory>

#include "pros/apix.h"

#include "ghost_serial/base_interfaces/v5_serial_base.hpp"
#include "ghost_serial/serial_utils/bitmasks.hpp"

namespace ghost_v5{

    class V5SerialNode
    {
    public:
        V5SerialNode(std::string msg_start_seq, int msg_len, bool use_checksum);
        ~V5SerialNode();

        void initSerial();
        bool readV5ActuatorUpdate();
        void writeV5StateUpdate();

    private:
        void updateActuatorCommands(unsigned char buffer[]);

        // Config Params
        int max_msg_len_;
        bool using_reader_thread_;

        // Serial Interface
        std::unique_ptr<ghost_serial::V5SerialBase> serial_base_interface_;
        std::vector<unsigned char> new_msg_;

        // Reader Thread
        std::unique_ptr<pros::Task> reader_thread_;
        std::atomic_bool reader_thread_init_;

        // Msg Config
        int actuator_command_msg_len_;
        int state_update_msg_len_;
    };
} // namespace ghost_v5
#endif // GHOST_V5__V5_SERIAL_NODE_HPP