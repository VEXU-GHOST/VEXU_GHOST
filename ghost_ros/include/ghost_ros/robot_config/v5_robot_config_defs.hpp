#ifndef V5_ROBOT_CONFIG_DEFS_HPP
#define V5_ROBOT_CONFIG_DEFS_HPP

// Macro for visually pleasant motor config (also sets string motor name from enum variable)
#define START_MOTORS const std::map<v5_motor_id_enum, std::tuple<bool, std::string, GhostMotorConfig>> motor_config_id_map{
#define ADD_MOTOR(id, reversed, config) {id, std::tuple<bool, std::string, GhostMotorConfig>(reversed, #id, config)},
#define END_MOTORS };

#define START_ENCODERS const std::map<v5_sensor_id_enum, std::tuple<std::string, bool>> encoder_config_id_map{
#define ADD_ENCODER(id, reversed) {id, std::tuple<std::string, bool>(#id, reversed)},
#define END_ENCODERS };

namespace ghost_v5_config
{
    enum ghost_gearset{
        GEARSET_100,
        GEARSET_200,
        GEARSET_600,
        GEARSET_3600
    };

    enum ghost_brake_mode{
        BRAKE_MODE_COAST,
        BRAKE_MODE_BRAKE,
        BRAKE_MODE_HOLD,
        BRAKE_MODE_INVALID
    };

    enum ghost_encoder_unit{
        ENCODER_DEGREES,
        ENCODER_ROTATIONS,
        ENCODER_COUNTS,
        ENCODER_INVALID
    };

    struct GhostMotorConfig
    {
        // PROS Motor
        ghost_encoder_unit motor__encoder_units{ghost_encoder_unit::ENCODER_DEGREES};
        float motor__nominal_free_speed{120};
        float motor__stall_torque{3.6};
        float motor__free_current{0.14};
        float motor__stall_current{4.25};
        float motor__max_voltage{12};
        ghost_gearset motor__gear_ratio{ghost_gearset::GEARSET_600};
        ghost_brake_mode motor__brake_mode{ghost_brake_mode::BRAKE_MODE_COAST};

        // 2nd Order Velocity Filter
        float filter__cutoff_frequency{100.0};
        float filter__damping_ratio{0.707};
        float filter__timestep{0.01};

        // FF-PD Controller
        float ctl__pos_gain{0.0};
        float ctl__vel_gain{10.0};
        float ctl__ff_vel_gain{1.0};
        float ctl__ff_voltage_gain{1.0};
        float ctl__ff_torque_gain{0.0};

        // Limit Instant Voltage Change
        float motor__torque_limit_norm{1.0};
    };
}
#endif // V5_ROBOT_CONFIG_DEFS_HPP