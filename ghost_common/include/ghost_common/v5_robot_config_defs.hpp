#pragma once
#include <unordered_map>
#include <vector>
#include <string>
namespace ghost_v5_config
{
    enum ghost_gearset
    {
        GEARSET_100,
        GEARSET_200,
        GEARSET_600,
        GEARSET_3600
    };

    enum ghost_brake_mode
    {
        BRAKE_MODE_COAST,
        BRAKE_MODE_BRAKE,
        BRAKE_MODE_HOLD,
        BRAKE_MODE_INVALID
    };

    enum ghost_encoder_unit
    {
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

        // FF-PD Controllermotor__encoder_units
        float ctl__pos_gain{0.0};
        float ctl__vel_gain{10.0};
        float ctl__ff_vel_gain{1.0};
        float ctl__ff_voltage_gain{1.0};
        float ctl__ff_torque_gain{0.0};

        // Limit Instant Voltage Change
        float motor__torque_limit_norm{1.0};
    };
    struct motor_access_helper
    {
        motor_access_helper(int port_init, bool reversed_init, GhostMotorConfig config_init)
        {
            port = port_init;
            reversed = reversed_init;
            config = config_init;
        };

        int port;
        bool reversed;
        GhostMotorConfig config;
    };

    struct encoder_access_helper
    {
        encoder_access_helper(int port_init, bool reversed_init)
        {
            port = port_init;
            reversed = reversed_init;
        };

        int port;
        bool reversed;
    };

    extern const std::unordered_map<std::string, motor_access_helper> motor_config_map;
    extern const std::unordered_map<std::string, encoder_access_helper> encoder_config_map;

    // Serial Msg Config
    extern const int actuator_cmd_extra_byte_count;
    extern const int actuator_update_packet_byte_size;
    extern const int motor_sensor_packet_byte_size;
    extern const int encoder_sensor_packet_byte_size;
    extern const int sensor_update_extra_byte_count;

    const int get_actuator_command_msg_len();
    const int get_sensor_update_msg_len();

    // Maps device enum to device_name
    extern const std::unordered_map<int, std::string> device_names;
}

// Macro for visually pleasant motor config (also sets string motor name from enum variable)
#define START_MOTORS                                                            \
    const std::unordered_map<std::string, motor_access_helper> motor_config_map \
    {
#define ADD_MOTOR(name, port, reversed, config) {name, motor_access_helper(port, reversed, config)},
#define END_MOTORS \
    }              \
    ;

#define START_ENCODERS                                                              \
    const std::unordered_map<std::string, encoder_access_helper> encoder_config_map \
    {
#define ADD_ENCODER(name, port, reversed) {name, encoder_access_helper(port, reversed)},
#define END_ENCODERS \
    }                \
    ;