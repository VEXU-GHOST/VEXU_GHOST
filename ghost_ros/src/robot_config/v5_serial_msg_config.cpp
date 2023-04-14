#include "ghost_ros/robot_config/v5_serial_msg_config.hpp"

namespace ghost_v5_config{
    // Defines order to pack Jetson -> V5 serial msg
    const std::vector<std::pair<v5_motor_id_enum, bool>> actuator_command_config = {
        std::pair<v5_motor_id_enum, bool>(DRIVE_LEFT_FRONT_MOTOR,       false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_LEFT_BACK_MOTOR,        false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_RIGHT_FRONT_MOTOR,      false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_RIGHT_BACK_MOTOR,       false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_BACK_LEFT_REAR_MOTOR,   false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_BACK_LEFT_FRONT_MOTOR,  false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_BACK_RIGHT_REAR_MOTOR,  false),
        std::pair<v5_motor_id_enum, bool>(DRIVE_BACK_RIGHT_FRONT_MOTOR, false),
        std::pair<v5_motor_id_enum, bool>(INTAKE_MOTOR_1,               false),
        std::pair<v5_motor_id_enum, bool>(INTAKE_MOTOR_2,               false),
        std::pair<v5_motor_id_enum, bool>(INDEXER_MOTOR,                false),
        std::pair<v5_motor_id_enum, bool>(SHOOTER_LEFT_MOTOR,           false),
        std::pair<v5_motor_id_enum, bool>(SHOOTER_RIGHT_MOTOR,          false),
    };
    // Each motor has current, velocity, and voltage (position is added separately)
    const int actuator_update_packet_byte_size = 3 * 4;
    const int actuator_cmd_extra_byte_count = 4 + 4; // 4x Bytes for Msg Sequence ID + 4x Bytes for Motor Active Vector

    // Defines order to pack V5 -> Jetson serial msg
    const std::vector<v5_motor_id_enum> sensor_update_motor_config = {
        DRIVE_LEFT_FRONT_MOTOR, 
        DRIVE_LEFT_BACK_MOTOR,
        DRIVE_RIGHT_FRONT_MOTOR,
        DRIVE_RIGHT_BACK_MOTOR, 
        DRIVE_BACK_LEFT_REAR_MOTOR,
        DRIVE_BACK_LEFT_FRONT_MOTOR,
        DRIVE_BACK_RIGHT_REAR_MOTOR,
        DRIVE_BACK_RIGHT_FRONT_MOTOR,
        INTAKE_MOTOR_1,
        INTAKE_MOTOR_2,
        INDEXER_MOTOR,
        SHOOTER_LEFT_MOTOR,
        SHOOTER_RIGHT_MOTOR,
    };

    const std::vector<v5_sensor_id_enum> sensor_update_sensor_config = {
        STEERING_LEFT_ENCODER,
        STEERING_RIGHT_ENCODER,
        STEERING_BACK_ENCODER,
    };

    // Each motor reports angle_degrees, velocity_rpm, voltage_mv, current_ma, power_w, temp_c  
    const int motor_sensor_packet_byte_size = 6 * 4; 
    const int encoder_sensor_packet_byte_size = 2 * 4; 

    // 4x int32 Joystick Channels, 2x bytes of btns/competition modes, 4x Bytes of port status, 4x Bytes of Msg Sequence ID
    const int sensor_update_extra_byte_count = 4*4 + 2 + 4 + 4;

    // Maps device enum to device_name
    const std::map<int, std::string> device_names{
        {DRIVE_LEFT_FRONT_MOTOR,        "DRIVE_LEFT_FRONT_MOTOR"},
        {DRIVE_LEFT_BACK_MOTOR,         "DRIVE_LEFT_BACK_MOTOR"},
        {DRIVE_RIGHT_FRONT_MOTOR,       "DRIVE_RIGHT_FRONT_MOTOR"},
        {DRIVE_RIGHT_BACK_MOTOR,        "DRIVE_RIGHT_BACK_MOTOR"},
        {DRIVE_BACK_LEFT_REAR_MOTOR,    "DRIVE_BACK_LEFT_REAR_MOTOR"},
        {DRIVE_BACK_RIGHT_REAR_MOTOR,   "DRIVE_BACK_RIGHT_REAR_MOTOR"},
        {DRIVE_BACK_LEFT_FRONT_MOTOR,   "DRIVE_BACK_LEFT_FRONT_MOTOR"},
        {DRIVE_BACK_RIGHT_FRONT_MOTOR,  "DRIVE_BACK_RIGHT_FRONT_MOTOR"},
        {INTAKE_MOTOR_1,                "INTAKE_MOTOR_1"},
        {INTAKE_MOTOR_2,                "INTAKE_MOTOR_2"},
        {INDEXER_MOTOR,                 "INDEXER_MOTOR"},
        {SHOOTER_LEFT_MOTOR,            "SHOOTER_LEFT_MOTOR"},
        {SHOOTER_RIGHT_MOTOR,           "SHOOTER_RIGHT_MOTOR"},
        {STEERING_LEFT_ENCODER,         "STEERING_LEFT_ENCODER"},
        {STEERING_RIGHT_ENCODER,        "STEERING_RIGHT_ENCODER"},
        {STEERING_BACK_ENCODER,         "STEERING_BACK_ENCODER"},
    };

    int get_actuator_command_msg_len(){
        int msg_len_ = actuator_update_packet_byte_size * actuator_command_config.size();
        for (auto &pair : actuator_command_config)
        {
            msg_len_ += (pair.second) ? 4 : 0; // Add four bytes for each motor using position control
        }
        msg_len_ += actuator_cmd_extra_byte_count;
        return msg_len_;
    }

    int get_sensor_update_msg_len(){
        int msg_len_ = 
			sensor_update_motor_config.size() * motor_sensor_packet_byte_size +
			sensor_update_sensor_config.size() * encoder_sensor_packet_byte_size;
		msg_len_ += ghost_v5_config::sensor_update_extra_byte_count;
        return msg_len_;
    }
}