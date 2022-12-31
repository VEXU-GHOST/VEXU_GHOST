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
        bool motor__reversed;
        pros::motor_encoder_units_e motor__encoder_units;
        float motor__nominal_free_speed;
        float motor__stall_torque;
        float motor__free_current;
        float motor__stall_current;
        float motor__max_voltage;
        float motor__gear_ratio;

        // 2nd Order Velocity Filter
        float filter__cutoff_frequency;
        float filter__damping_ratio;
        float filter__timestep;

        // FF-PD Controller
        float ctl__pos_gain;
        float ctl__vel_gain;
        float ctl__ff_vel_gain;
        float ctl__ff_voltage_gain;

        // Limit Instant Voltage Change
        float motor__torque_limit_norm;
    };

    enum control_mode_e{
        VOLTAGE_CONTROL,
        VELOCITY_CONTROL,
        POSITION_CONTROL,
    };

    class GhostMotor : public pros::Motor
    {
    public:
        GhostMotor(int motor_port, ghost_motor_config &config);

        void updateMotor();

        float getVelocityFilteredRPM(){
            return velocity_filter_.getCurrentState();
        }

        void setMotorCommand(float voltage, float velocity, float position);
        void setMotorCommand(float voltage, float velocity);
        void setMotorCommand(float voltage);

    private:
        void move_voltage_trq_lim(float voltage_mv);
    
        // Motor Config
        ghost_motor_config config_;
        std::map<int, pros::motor_gearset_e_t> RPM_TO_GEARING{
            {1, pros::E_MOTOR_GEAR_100},
            {2, pros::E_MOTOR_GEAR_200},
            {6, pros::E_MOTOR_GEAR_600}
        };

        // Velocity Filtering
        ghost_estimation::SecondOrderLowPassFilter velocity_filter_;
        float curr_vel_rpm_;

        // Motor Controller
        control_mode_e ctl_mode_;

        int32_t des_pos_encoder_;
        float des_vel_rpm_;
        float des_voltage_norm_;

        float cmd_voltage_mv_;

        // Motor Models
        ghost_control::DCMotorModel motor_model_;
    };

} // namespace ghost_v5

#endif // GHOST_V5__GHOST_MOTOR_HPP