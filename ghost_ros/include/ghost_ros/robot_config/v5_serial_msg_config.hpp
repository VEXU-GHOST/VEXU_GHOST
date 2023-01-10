
#ifndef GHOST_ROS__V5_SERIAL_MSG_CONFIG_HPP
#define GHOST_ROS__V5_SERIAL_MSG_CONFIG_HPP

#include <string>
#include <vector>
#include <map>

#include "v5_port_config.hpp"

namespace ghost_v5_config{

    // Defines order to pack Jetson -> V5 serial msg
    extern const std::vector<std::pair<v5_motor_id_enum, bool>> actuator_command_config;

    // Defines order to pack V5 -> Jetson serial msg
    extern const std::vector<v5_motor_id_enum> state_update_motor_config;
    extern const std::vector<v5_sensor_id_enum> state_update_sensor_config;

    // 4x int32 Joystick Channels, 2x bytes of btns/digital outs/competition modes, 4x Bytes of port status
    extern const int state_update_extra_byte_count;

    // Maps device enum to device_name
    extern const std::map<int, std::string> device_names;

}

#endif // GHOST_ROS__V5_SERIAL_MSG_CONFIG_HPP