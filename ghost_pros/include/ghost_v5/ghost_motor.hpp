#ifndef GHOST_V5__GHOST_MOTOR_HPP
#define GHOST_V5__GHOST_MOTOR_HPP

#include <map>
#include "pros/motors.hpp"
#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"
#include "ghost_control/models/dc_motor_model.hpp"

namespace ghost_v5
{

    struct ghost_motor_config
    {
        // PROS Motor
        int motor_port;
        int motor_gearing;
        bool motor_reversed;
        pros::motor_encoder_units_e motor_encoder_units;

        // 2nd Order Velocity Filter
        float filter_cutoff_frequency;
        float filter_damping_ratio;
        float filter_timestep;

        // FF-PD Controller
        float ctl_pos_gain;
        float ctl_vel_gain;

        // Limit Instant Voltage Change
        float motor_voltage_slew;
        
        // DC Motor Model
        float model_free_speed;
        float model_stall_torque;
        float model_free_current;
        float model_stall_current;
        float model_gear_ratio;
    };

    class GhostMotor : public pros::Motor
    {
    public:
        GhostMotor(const ghost_motor_config config);

        void updateMotor();
        void setMotorCommand(float voltage, float velocity, int32_t position);
        void setMotorCommand(float voltage, float velocity);

    private:
        // Motor Config
        ghost_motor_config config_;
        std::map<int, pros::motor_gearset_e> RPM_TO_GEARING{
            {100, pros::E_MOTOR_GEAR_100},
            {200, pros::E_MOTOR_GEAR_200},
            {600, pros::E_MOTOR_GEAR_600}
        };

        // Velocity Filtering
        ghost_estimation::SecondOrderLowPassFilter velocity_filter_;
        float curr_vel_rpm_;

        // Motor Controller
        float use_pos_ctl_;

        int32_t pos_des_ticks_;
        float vel_des_rpm_;
        float torque_des_nm_;

        int16_t cmd_voltage_mv_;

        // Motor Models
        ghost_control::DCMotorModel motor_model_;
    };

} // namespace ghost_v5

#endif // GHOST_V5__GHOST_MOTOR_HPP