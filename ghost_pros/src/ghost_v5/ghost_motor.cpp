#include "ghost_v5/ghost_motor.hpp"

namespace ghost_v5
{
    GhostMotor::GhostMotor(
        const ghost_motor_config config)
        : pros::Motor(
              config.motor_port,
              RPM_TO_GEARING[config.motor_gearing],
              config.motor_reversed,
              config.motor_encoder_units),
          velocity_filter_(
              config.filter_cutoff_frequency,
              config.filter_damping_ratio,
              config.filter_timestep),
          motor_model_(
              config.model_free_speed,
              config.model_stall_torque,
              config.model_free_current,
              config.model_stall_current,
              12.0,
              config.model_gear_ratio)
    {
        config_ = config;
    }

    void GhostMotor::updateMotor()
    {
        // Update Low Pass Filter with velocity measurement
        curr_vel_rpm_ = velocity_filter_.updateFilter(get_actual_velocity());

        // Update DC Motor Model
        motor_model_.setMotorSpeedRPM(curr_vel_rpm_);
        float ff_voltage_mv = motor_model_.getVoltageFromTorque(torque_des_nm_) * 1000;

        // Update motor
        cmd_voltage_mv_ =
            ff_voltage_mv +
            (vel_des_rpm_ - curr_vel_rpm_) * config_.ctl_vel_gain +
            (pos_des_ticks_ - get_position()) * config_.ctl_pos_gain * use_pos_ctl_;
    }

    void GhostMotor::setMotorCommand(float torque, float velocity, int32_t position)
    {
        use_pos_ctl_ = 1.0;

        pos_des_ticks_ = position;
        vel_des_rpm_ = velocity;
        torque_des_nm_ = torque;
    }

    void GhostMotor::setMotorCommand(float torque, float velocity)
    {
        use_pos_ctl_ = 0.0;

        vel_des_rpm_ = velocity;
        torque_des_nm_ = torque;
    }

} // namespace ghost_motor