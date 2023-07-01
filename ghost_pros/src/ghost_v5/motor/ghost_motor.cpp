#include "ghost_v5/motor/ghost_motor.hpp"

#include "pros/apix.h"
#include "pros/error.h"

using ghost_v5_config::GhostMotorConfig;
using ghost_v5_config::ghost_brake_mode;
using ghost_v5_config::ghost_gearset;
using ghost_v5_config::ghost_encoder_unit;
namespace ghost_v5
{
    GhostMotor::GhostMotor(
        int motor_port,
        bool reversed,
        GhostMotorConfig &config)
        : pros::Motor(
              motor_port,
              pros::E_MOTOR_GEARSET_INVALID,
              reversed,
              pros::E_MOTOR_ENCODER_INVALID),
          velocity_filter_(
              config.filter__cutoff_frequency,
              config.filter__damping_ratio,
              config.filter__timestep),
          motor_model_(
              config.motor__nominal_free_speed,
              config.motor__stall_torque,
              config.motor__free_current,
              config.motor__stall_current,
              config.motor__max_voltage,
              config.motor__gear_ratio),
          motor_is_3600_cart_{(config_.motor__gear_ratio == ghost_gearset::GEARSET_3600)},
          device_connected_{false},
          des_voltage_norm_{0.0},
          des_vel_rpm_{0.0},
          des_pos_encoder_{0},
          cmd_voltage_mv_{0.0},
          position_active_{false},
          velocity_active_{false},
          torque_active_{false},
          voltage_active_{false}
    {
        config_ = config;
        trq_lim_norm_ = config_.motor__torque_limit_norm;

        // Set Motor Config w PROS Enum values
        set_gearing(RPM_TO_GEARING_MAP.at(config_.motor__gear_ratio));
        set_brake_mode(GHOST_BRAKE_MODE_MAP.at(config_.motor__brake_mode));
        set_encoder_units(GHOST_ENCODER_UNIT_MAP.at(config_.motor__encoder_units));

        if(config.motor__gear_ratio == ghost_gearset::GEARSET_3600){
            motor_is_3600_cart_ = true;
        }
    }

    void GhostMotor::updateMotor()
    {
        // Get Motor Velocity, checking for errors
        double raw_vel = get_actual_velocity();
        device_connected_ = (raw_vel != PROS_ERR_F);
        if(raw_vel == PROS_ERR_F){
            raw_vel = 0.0;
        }
        
        // Adjust reported velocity for direct drive motor gearing
        auto true_vel = (motor_is_3600_cart_) ?  6*raw_vel : raw_vel;
        
        // Update Low Pass Filter with velocity measurement
        curr_vel_rpm_ = velocity_filter_.updateFilter(true_vel);

        // Update DC Motor Model
        motor_model_.setMotorSpeedRPM(curr_vel_rpm_);

        // Calculate control inputs
        float position_feedback = (position_active_) ? (des_pos_encoder_ - get_position()) * config_.ctl__pos_gain : 0.0;
        float velocity_feedforward = (velocity_active_) ? motor_model_.getVoltageFromVelocityMillivolts(des_vel_rpm_) * config_.ctl__ff_vel_gain : 0.0;
        float velocity_feedback = (velocity_active_) ? (des_vel_rpm_ - curr_vel_rpm_) * config_.ctl__vel_gain : 0.0;
        float torque_feedforward = (torque_active_) ? motor_model_.getVoltageFromTorqueMillivolts(des_torque_nm_) * config_.ctl__ff_torque_gain : 0.0;
        float voltage_feedforward = (voltage_active_) ? des_voltage_norm_ * config_.motor__max_voltage * 1000 * config_.ctl__ff_voltage_gain : 0.0;

        // Set voltage command based on control mode
        cmd_voltage_mv_ = voltage_feedforward + torque_feedforward + velocity_feedforward + velocity_feedback + position_feedback;

        if(position_active_ || velocity_active_ || torque_active_ || voltage_active_){
            move_voltage(cmd_voltage_mv_);
        }
        else{
            set_current_limit(0);
            move_voltage(0);
        }

        // Motor control mode must be constantly refreshed
        position_active_ = false;
        velocity_active_ = false;
        torque_active_ = false;
        voltage_active_ = false;
    }

    /*
    This was used for defected V5 REV 10 motors, no longer really relevant.
    */
    void GhostMotor::move_voltage_trq_lim(float voltage_mv)
    {
        // Normalize voltage command from millivolts
        double voltage_normalized = voltage_mv / config_.motor__max_voltage / 1000.0;

        // Normalize velocity by nominal free speed
        double curr_vel_normalized = curr_vel_rpm_ / (config_.motor__nominal_free_speed * config_.motor__gear_ratio);

        // Motor torque is proportional to armature current, which is approximated by difference between back EMF
        // and driving voltage assuming constant resistance.
        // Limiting voltage difference prevents voltage spikes and prolongs motor life when changing speed rapidly.
        double cmd;
        if (fabs(curr_vel_normalized - voltage_normalized) > trq_lim_norm_)
        {
            double sign = (curr_vel_normalized - voltage_normalized > 0) ? 1.0 : -1.0;
            cmd = curr_vel_normalized - sign * fabs(trq_lim_norm_);
        }
        else
        {
            cmd = voltage_normalized;
        }

        // Limit cmd to voltage bounds
        cmd = std::min(cmd, config_.motor__max_voltage * 1000.0);
        cmd = std::max(cmd, -config_.motor__max_voltage * 1000.0);

        // Set motor
        move_voltage(cmd * config_.motor__max_voltage * 1000);
    }

} // namespace ghost_motor