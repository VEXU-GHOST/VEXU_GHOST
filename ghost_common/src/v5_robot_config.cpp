/*
For each motor configuration, we define a struct
*/


#include "ghost_common/v5_robot_config_defs.hpp"

namespace ghost_v5_config
{
    // Drive Motor Config
    GhostMotorConfig drive_motor_config = {
        .motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_600,
        .motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_COAST,
        .ctl__vel_gain = 17.5,  // RPM -> mV
        .ctl__ff_vel_gain = 1.1,
    };

    GhostMotorConfig endgame_motor_config = {
        .motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_200,
    };

    // Intake Motor Config
    GhostMotorConfig intake_motor_config = {
        .motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_600,
        .motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_BRAKE,
        .ctl__vel_gain = 25.0,
        .ctl__ff_vel_gain = 1.2,
    };

    // Indexer Motor Config
    GhostMotorConfig indexer_motor_config = {
        .motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_600,
        .motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_BRAKE,
        .filter__cutoff_frequency = 50.0,
        .ctl__pos_gain = 0.0,
        .ctl__vel_gain = 0.0,
        .ctl__ff_vel_gain = 0.0,
    };

    START_MOTORS
    ADD_MOTOR("DRIVE_LEFT_FRONT_MOTOR",   1,  false,  drive_motor_config)
    ADD_MOTOR("DRIVE_LEFT_BACK_MOTOR",    2,  false,  drive_motor_config)
    ADD_MOTOR("DRIVE_RIGHT_FRONT_MOTOR",  3,  false,  drive_motor_config)
    ADD_MOTOR("DRIVE_RIGHT_BACK_MOTOR",   4,  false,  drive_motor_config)
    END_MOTORS

    START_ENCODERS
    // ADD_ENCODER(STEERING_LEFT_ENCODER, false)
    // ADD_ENCODER(STEERING_RIGHT_ENCODER, false)
    // ADD_ENCODER(STEERING_BACK_ENCODER, false)
    END_ENCODERS
}