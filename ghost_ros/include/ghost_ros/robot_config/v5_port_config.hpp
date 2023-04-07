#ifndef GHOST_ROS__V5_PORT_CONFIG_HPP
#define GHOST_ROS__V5_PORT_CONFIG_HPP

#include <string>
#include <map>

namespace ghost_v5_config
{
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
        TILTER_MOTOR                    = 14,
        INTAKE_MOTOR                    = 19,
        SHOOTER_RIGHT_MOTOR             = 15,
        SHOOTER_LEFT_MOTOR              = 18,
        INDEXER_MOTOR                   = 20
    };

    enum v5_sensor_id_enum
    {
        STEERING_LEFT_ENCODER           = 1,
        STEERING_RIGHT_ENCODER          = 2,
        STEERING_BACK_ENCODER           = 12,
        TILTER_ENCODER                  = 11
    };

    enum v5_pneumatic_id_enum
    {
        FLYWHEEL_TILT   = 1,
        INDEXER_ROOF    = 2,
        ENDGAME         = 3,
    };
    
} // ghost_v5_config

#endif // GHOST_ROS__V5_PORT_CONFIG_HPP