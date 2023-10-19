#include <cmath>
#include "ghost_interfaces/v5_robot_config_defs.hpp"

namespace ghost_interfaces {

// Each motor has position, velocity, torque, voltage, and current
// Each motor has four control active flags, add one byte per motor for flags
constexpr int actuator_update_packet_byte_size = 5 * 4 + 1;
constexpr int actuator_cmd_extra_byte_count = 2; // 2x Bytes for Msg Sequence ID + 3x Bytes for Motor Active Vector

// Each motor reports position_degrees, velocity_rpm, voltage_mv, torque_nm, current_ma, power_w, temp_c
constexpr int motor_sensor_packet_byte_size = 7 * 4;

// Each encoder reports position and velocity
constexpr int encoder_sensor_packet_byte_size = 2 * 4;

// 4x int32 Joystick Channels, 2x bytes of btns/competition modes, 4x Bytes of port status, 4x Bytes of Msg Sequence ID
constexpr int sensor_update_extra_byte_count = 4 * 4 + 2 + 4 + 4;

// Define full msg lengths for bi-directional serial

// TODO(maxxwilson): Loading motor and encoder sizes needs to come from the generated compile-time config

// const int actuator_command_msg_len = actuator_update_packet_byte_size * motor_config_map.size() + actuator_cmd_extra_byte_count;
// const int sensor_update_msg_len = motor_config_map.size() * motor_sensor_packet_byte_size +
//                                   encoder_config_map.size() * encoder_sensor_packet_byte_size +
//                                   ghost_v5_config::sensor_update_extra_byte_count;

}