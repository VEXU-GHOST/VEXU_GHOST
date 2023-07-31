
#ifndef GHOST_ROS__V5_SERIAL_MSG_CONFIG_HPP
#define GHOST_ROS__V5_SERIAL_MSG_CONFIG_HPP

#include <string>
#include <vector>
#include <map>

#include "v5_robot_config.hpp"

namespace ghost_v5_config{

    // Defines order to pack Jetson -> V5 serial msg
    extern const std::vector<v5_motor_id_enum> actuator_command_config;
    extern const int actuator_cmd_extra_byte_count;
    extern const int actuator_update_packet_byte_size;
    extern const int motor_sensor_packet_byte_size;
    extern const int encoder_sensor_packet_byte_size;

    // Defines order to pack V5 -> Jetson serial msg
    extern const std::vector<v5_motor_id_enum> sensor_update_motor_config;
    extern const std::vector<v5_sensor_id_enum> sensor_update_sensor_config;

    // 4x int32 Joystick Channels, 2x bytes of btns/digital outs/competition modes, 4x Bytes of port status
    extern const int sensor_update_extra_byte_count;

    // Maps device enum to device_name
    extern const std::map<int, std::string> device_names;

    int get_actuator_command_msg_len();
    int get_sensor_update_msg_len();

}

#endif // GHOST_ROS__V5_SERIAL_MSG_CONFIG_HPP