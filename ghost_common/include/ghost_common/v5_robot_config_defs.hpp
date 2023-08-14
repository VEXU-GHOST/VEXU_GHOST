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
        GEARSET_600
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
        // These three map 1:1 to their PROS counterpart on the V5 Side.
        ghost_encoder_unit motor__encoder_units{ghost_encoder_unit::ENCODER_DEGREES};
        ghost_gearset motor__gear_ratio{ghost_gearset::GEARSET_600};
        ghost_brake_mode motor__brake_mode{ghost_brake_mode::BRAKE_MODE_COAST};

        // FF-PD Controller
        // pos_gain and vel_gain are standard PD control
        // ff_vel_gain takes a velocity setpoint and scales it to estimate the required open-loop voltage
        // Ideally this is 1.0. If your motor runs faster than nominal, increase this a bit.
        // If you zero the other gains, you can tune this by sending a velocity and tweaking until true velocity matches.
        // ff_torque_gain does the same as above but for torque. Controlling torque with voltage is not very accurate, fyi.
        float ctl__pos_gain{0.0};
        float ctl__vel_gain{10.0};
        float ctl__ff_vel_gain{1.0};
        float ctl__ff_torque_gain{0.0};

        // 2nd Order Low Pass Filter for Motor Velocity
        // If you aren't familiar with LPFs, default tuning is probably fine for any VEX system.
        // If you want it smoother, lower the cutoff frequency.
        // "Ideal setting" is 10 / ((time for system to go from zero to full speed given max voltage) * 0.632)
        float filter__cutoff_frequency{100.0};
        float filter__damping_ratio{0.707}; // Don't Change
        float filter__timestep{0.01};       // Don't Change

        // This configures an 11W V5 Motor.
        // NOTE: The gear ratio is set by the ghost_gearset param above and scaling is applied within the motor class.
        // This means, YOU PROBABLY SHOULDN'T TOUCH ANY OF THESE.
        float motor__nominal_free_speed{120};   // Don't Change
        float motor__stall_torque{3.6};         // Don't Change
        float motor__free_current{0.14};        // Don't Change
        float motor__stall_current{4.25};       // Don't Change
        float motor__max_voltage{12};           // Don't Change
    };

    // Helper classes for accessing motor/encoders from their config maps
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

    int get_actuator_command_msg_len();
    int get_sensor_update_msg_len();
}