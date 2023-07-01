/*
For each motor configuration, we define a struct
*/


#include "ghost_ros/robot_config/v5_robot_config.hpp"

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

    // Shooter Motor Config
    GhostMotorConfig shooter_motor_config = {
        .motor__gear_ratio = ghost_v5_config::ghost_gearset::GEARSET_3600,
        .motor__brake_mode = ghost_v5_config::ghost_brake_mode::BRAKE_MODE_BRAKE,
        .ctl__vel_gain = 20.0,  // RPM -> mV
        .ctl__ff_vel_gain = 1.0,
    };

    START_MOTORS
    ADD_MOTOR(DRIVE_LEFT_FRONT_MOTOR,       false,  drive_motor_config)
    ADD_MOTOR(DRIVE_LEFT_BACK_MOTOR,        false,  drive_motor_config)
    ADD_MOTOR(DRIVE_RIGHT_FRONT_MOTOR,      false,  drive_motor_config)
    ADD_MOTOR(DRIVE_RIGHT_BACK_MOTOR,       false,  drive_motor_config)
    ADD_MOTOR(DRIVE_BACK_LEFT_REAR_MOTOR,   false,  drive_motor_config)
    ADD_MOTOR(DRIVE_BACK_LEFT_FRONT_MOTOR,  true,   drive_motor_config)
    ADD_MOTOR(DRIVE_BACK_RIGHT_REAR_MOTOR,  false,  drive_motor_config)
    ADD_MOTOR(DRIVE_BACK_RIGHT_FRONT_MOTOR, true,   drive_motor_config)
    ADD_MOTOR(INTAKE_MOTOR_1,               false,  intake_motor_config)
    ADD_MOTOR(INTAKE_MOTOR_2,               true,   intake_motor_config)
    ADD_MOTOR(SHOOTER_RIGHT_MOTOR,          false,  shooter_motor_config)
    ADD_MOTOR(SHOOTER_LEFT_MOTOR,           true,   shooter_motor_config)
    ADD_MOTOR(INDEXER_MOTOR,                true,   indexer_motor_config)
    ADD_MOTOR(ENDGAME_MOTOR,                false,  endgame_motor_config)
    END_MOTORS

    START_ENCODERS
    ADD_ENCODER(STEERING_LEFT_ENCODER, false)
    ADD_ENCODER(STEERING_RIGHT_ENCODER, false)
    ADD_ENCODER(STEERING_BACK_ENCODER, false)
    END_ENCODERS
}