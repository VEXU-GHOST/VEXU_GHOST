#ifndef GHOST_ROS__V5_PORT_CONFIG_HPP
#define GHOST_ROS__V5_PORT_CONFIG_HPP

#include <string>
#include <map>

namespace ghost_v5_config
{
    enum v5_motor_id_enum
    {
        DRIVE_LEFT_FRONT_MOTOR      = 11,
        DRIVE_LEFT_BACK_MOTOR       = 12,
        DRIVE_RIGHT_FRONT_MOTOR     = 13,
        DRIVE_RIGHT_BACK_MOTOR      = 14,
        DRIVE_BACK_LEFT_1_MOTOR     = 15,
        DRIVE_BACK_RIGHT_1_MOTOR    = 16,
        DRIVE_BACK_LEFT_2_MOTOR     = 17,
        DRIVE_BACK_RIGHT_2_MOTOR    = 18,
        TURRET_MOTOR                = 1,
        INTAKE_MOTOR                = 2,
        INDEXER_MOTOR               = 3,
        SHOOTER_LEFT_MOTOR          = 4,
        SHOOTER_RIGHT_MOTOR         = 5
    };

    enum v5_sensor_id_enum
    {
        STEERING_LEFT_ENCODER       = 20,
        STEERING_RIGHT_ENCODER      = 19,
        STEERING_BACK_ENCODER       = 9,
    };
    
} // ghost_v5_config

#endif // GHOST_ROS__V5_PORT_CONFIG_HPP