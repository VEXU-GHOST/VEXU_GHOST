#ifndef GHOST_ROS__V5_PORT_CONFIG_HPP
#define GHOST_ROS__V5_PORT_CONFIG_HPP

#include <string>
#include <map>

namespace ghost_v5_config
{
    enum v5_motor_id_enum
    {
        DRIVE_LEFT_FRONT_MOTOR      = 1,
        DRIVE_LEFT_BACK_MOTOR       = 2,
        DRIVE_RIGHT_FRONT_MOTOR     = 3,
        DRIVE_RIGHT_BACK_MOTOR      = 4,
        DRIVE_BACK_LEFT_1_MOTOR     = 5,
        DRIVE_BACK_RIGHT_1_MOTOR    = 6,
        DRIVE_BACK_LEFT_2_MOTOR     = 7,
        DRIVE_BACK_RIGHT_2_MOTOR    = 8,
        TURRET_MOTOR                = 9,
        INTAKE_MOTOR                = 10,
        INDEXER_MOTOR               = 11,
        SHOOTER_LEFT_MOTOR          = 12,
        SHOOTER_RIGHT_MOTOR         = 13
    };

    enum v5_sensor_id_enum
    {
        STEERING_LEFT_ENCODER       = 17,
        STEERING_RIGHT_ENCODER      = 18,
        STEERING_BACK_ENCODER       = 19,
    };
    
} // ghost_v5_config

#endif // GHOST_ROS__V5_PORT_CONFIG_HPP