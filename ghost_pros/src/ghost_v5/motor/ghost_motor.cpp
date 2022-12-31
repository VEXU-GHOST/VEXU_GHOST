#include "ghost_v5/motor/ghost_motor.hpp"

#include <iostream>

namespace ghost_v5
{
    GhostMotor::GhostMotor(
        int motor_port,
        ghost_motor_config &config)
        : pros::Motor(
              motor_port,
              pros::E_MOTOR_GEARSET_INVALID,
              config.motor__reversed,
              config.motor__encoder_units),
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
              config.motor__gear_ratio)
    {
        config_ = config;
        
        // Doing this in the constructor causes data abort exception
        set_gearing(RPM_TO_GEARING[config_.motor__gear_ratio]);
    }

    void GhostMotor::updateMotor()
    {
        // Update Low Pass Filter with velocity measurement
        curr_vel_rpm_ = velocity_filter_.updateFilter(get_actual_velocity());

        // Update DC Motor Model
        motor_model_.setMotorSpeedRPM(curr_vel_rpm_);

        // Calculate control inputs
        float voltage_feedforward = des_voltage_norm_ * config_.motor__max_voltage * 1000 * config_.ctl__ff_voltage_gain;
        float velocity_feedforward = motor_model_.getVoltageFromVelocityMillivolts(des_vel_rpm_) * config_.ctl__ff_vel_gain;
        float velocity_feedback = (des_vel_rpm_ - curr_vel_rpm_) * config_.ctl__vel_gain;
        float position_feedback = (des_pos_encoder_ - get_position()) * config_.ctl__pos_gain;

        // Set voltage command based on control mode
        switch(ctl_mode_){
            case control_mode_e::POSITION_CONTROL:
                cmd_voltage_mv_ = voltage_feedforward + velocity_feedforward + velocity_feedback + position_feedback;
            break;

            case control_mode_e::VELOCITY_CONTROL:
                cmd_voltage_mv_ = voltage_feedforward + velocity_feedforward + velocity_feedback;
            break;

            case control_mode_e::VOLTAGE_CONTROL:
                cmd_voltage_mv_ = voltage_feedforward;
            break;
        }

        // Set motor voltage w/ torque limiting (limiting change in voltage)
        move_voltage_trq_lim(cmd_voltage_mv_);
    }

    void GhostMotor::move_voltage_trq_lim(float voltage_mv){
        // Normalize voltage command from millivolts
        double voltage_normalized = voltage_mv / config_.motor__max_voltage / 1000.0;

        // Normalize velocity by nominal free speed
        double curr_vel_normalized = curr_vel_rpm_ / (config_.motor__nominal_free_speed * config_.motor__gear_ratio);

        // Motor torque is proportional to armature current, which is approximated by difference between back EMF
        // and driving voltage assuming constant resistance.
        // Limiting voltage difference prevents voltage spikes and prolongs motor life when changing speed rapidly.
        double cmd;
        if(fabs(curr_vel_normalized - voltage_normalized) > config_.motor__torque_limit_norm){
            double sign = (curr_vel_normalized - voltage_normalized > 0) ? 1.0 : -1.0;
            cmd = curr_vel_normalized - sign*fabs(config_.motor__torque_limit_norm);
        }
        else{
            cmd = voltage_normalized;
        }
        
        // Limit cmd to voltage bounds
        cmd = std::min(cmd, config_.motor__max_voltage*1000.0);
        cmd = std::max(cmd, -config_.motor__max_voltage*1000.0);

        // Set motor
        move_voltage(cmd*config_.motor__max_voltage*1000);
    }

    void GhostMotor::setMotorCommand(float voltage, float velocity, float position)
    {
        ctl_mode_ = control_mode_e::POSITION_CONTROL;

        des_pos_encoder_ = position;
        des_vel_rpm_ = velocity;
        des_voltage_norm_ = voltage;
    }

    void GhostMotor::setMotorCommand(float voltage, float velocity)
    {
        ctl_mode_ = control_mode_e::VELOCITY_CONTROL;

        des_vel_rpm_ = velocity;
        des_voltage_norm_ = voltage;
    }

    void GhostMotor::setMotorCommand(float voltage){
        ctl_mode_ = control_mode_e::VOLTAGE_CONTROL;
        des_voltage_norm_ = voltage;
    }

} // namespace ghost_motor