#include "ghost_common/v5_robot_config_defs.hpp"
#include <cmath>

namespace ghost_v5_config
{
    // Each motor has position, velocity, torque, voltage, and current
    // Each motor has four control active flags, add one byte per motor for flags
    constexpr int actuator_update_packet_byte_size = 5 * 4 + 1;
    constexpr int actuator_cmd_extra_byte_count = 2; // 2x Bytes for Msg Sequence ID + 3x Bytes for Motor Active Vector

    // Each motor reports angle_degrees, velocity_rpm, voltage_mv, torque_nm, current_ma, power_w, temp_c
    constexpr int motor_sensor_packet_byte_size = 7 * 4;

    // Each encoder reports position and velocity
    constexpr int encoder_sensor_packet_byte_size = 2 * 4;

    // 4x int32 Joystick Channels, 2x bytes of btns/competition modes, 4x Bytes of port status, 4x Bytes of Msg Sequence ID
    constexpr int sensor_update_extra_byte_count = 4 * 4 + 2 + 4 + 4;

    const int get_actuator_command_msg_len()
    {
        int msg_len_ = actuator_update_packet_byte_size * motor_config_map.size();
        msg_len_ += actuator_cmd_extra_byte_count;
        return msg_len_;
    }

    const int get_sensor_update_msg_len()
    {
        int msg_len_ =
            motor_config_map.size() * motor_sensor_packet_byte_size +
            encoder_config_map.size() * encoder_sensor_packet_byte_size;
        msg_len_ += ghost_v5_config::sensor_update_extra_byte_count;
        return msg_len_;
    }
}