#ifndef GHOST_V5__GHOST_MOTOR_HPP
#define GHOST_V5__GHOST_MOTOR_HPP

#include <map>
#include "pros/motors.hpp"
#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"
#include "ghost_control/models/dc_motor_model.hpp"

namespace ghost_v5
{
    struct GhostMotorConfig
    {
        // PROS Motor
        pros::motor_encoder_units_e motor__encoder_units{pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES};
        float motor__nominal_free_speed{120};
        float motor__stall_torque{3.6};
        float motor__free_current{0.14};
        float motor__stall_current{4.25};
        float motor__max_voltage{12};
        float motor__gear_ratio{600};
        pros::motor_brake_mode_e motor__brake_mode{pros::E_MOTOR_BRAKE_COAST};

        // 2nd Order Velocity Filter
        float filter__cutoff_frequency{75.0};
        float filter__damping_ratio{0.707};
        float filter__timestep{0.01};

        // FF-PD Controller
        float ctl__pos_gain{0.0};
        float ctl__vel_gain{10.0};
        float ctl__ff_vel_gain{1.0};
        float ctl__ff_voltage_gain{1.0};
        float ctl__rpm_deadband{0.0};

        // Limit Instant Voltage Change
        float motor__torque_limit_norm{1.0};
    };

    enum control_mode_e{
        VOLTAGE_CONTROL,
        VELOCITY_CONTROL,
        POSITION_CONTROL,
    };

    class GhostMotor : public pros::Motor
    {
    public:
        GhostMotor(int motor_port, bool reversed, GhostMotorConfig &config);

        void updateMotor();

        float getVoltageCommand(){
            return cmd_voltage_mv_;
        }

        float getVelocityFilteredRPM(){
            return velocity_filter_.getCurrentState();
        }

        bool getDeviceIsConnected(){
            return device_connected_;
        }

        void setActive(bool is_active){
            is_active_ = is_active;
        }

        void setMotorCommand(float voltage, float velocity, float position);
        void setMotorCommand(float voltage, float velocity);
        void setMotorCommand(float voltage);

    private:
        void move_voltage_trq_lim(float voltage_mv);
    
        // Motor Config
        GhostMotorConfig config_;
        std::map<int, pros::motor_gearset_e_t> RPM_TO_GEARING{
            {1, pros::E_MOTOR_GEAR_100},
            {2, pros::E_MOTOR_GEAR_200},
            {6, pros::E_MOTOR_GEAR_600},
            {36, pros::E_MOTOR_GEAR_600}, // PROS::Motor 
        };

        bool motor_is_3600_cart_;
        float trq_lim_norm_;

        // Velocity Filtering
        ghost_estimation::SecondOrderLowPassFilter velocity_filter_;
        float curr_vel_rpm_;
        bool device_connected_;

        // Motor Controller
        control_mode_e ctl_mode_;

        int32_t des_pos_encoder_;
        float des_vel_rpm_;
        float des_voltage_norm_;
        float cmd_voltage_mv_;
        float ctl_rpm_deadband_;
        bool is_active_;

        // Motor Models
        ghost_control::DCMotorModel motor_model_;
    };

} // namespace ghost_v5

#endif // GHOST_V5__GHOST_MOTOR_HPP