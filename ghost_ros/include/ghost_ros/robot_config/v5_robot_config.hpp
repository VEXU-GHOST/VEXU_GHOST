/*
This file defines the motor and sensor port mappings on the V5 Brain via enumerations.
This lets you refer to motors by name throughout the code.

We also 

*/
#ifndef GHOST_ROS__V5_PORT_CONFIG_HPP
#define GHOST_ROS__V5_PORT_CONFIG_HPP

#include <string>
#include <vector>
#include <tuple>
#include <map>

#include "ghost_ros/robot_config/v5_robot_config_defs.hpp"

namespace ghost_v5_config
{
    // Motor Ports
    enum v5_motor_id_enum
    {
        DRIVE_LEFT_FRONT_MOTOR          = 3,
        DRIVE_LEFT_BACK_MOTOR           = 4,
        DRIVE_RIGHT_FRONT_MOTOR         = 10,
        DRIVE_RIGHT_BACK_MOTOR          = 9,
        DRIVE_BACK_LEFT_REAR_MOTOR      = 6,
        DRIVE_BACK_LEFT_FRONT_MOTOR     = 5,
        DRIVE_BACK_RIGHT_REAR_MOTOR     = 7,
        DRIVE_BACK_RIGHT_FRONT_MOTOR    = 8,
        INTAKE_MOTOR_1                  = 19,
        INTAKE_MOTOR_2                  = 17,
        SHOOTER_RIGHT_MOTOR             = 15,
        ENDGAME_MOTOR                   = 14,
        SHOOTER_LEFT_MOTOR              = 18,
        INDEXER_MOTOR                   = 11
    };

    // Rotation Sensor Ports
    enum v5_sensor_id_enum
    {
        STEERING_LEFT_ENCODER           = 1,
        STEERING_RIGHT_ENCODER          = 2,
        STEERING_BACK_ENCODER           = 12,
    };

    extern const std::map<v5_motor_id_enum, std::tuple<bool, std::string, GhostMotorConfig>> motor_config_id_map;
    extern const std::map<v5_sensor_id_enum, std::tuple<std::string, bool>> encoder_config_id_map;
    
} // ghost_v5_config

#endif // GHOST_ROS__V5_PORT_CONFIG_HPP