#include "ghost_control/motor_controller.hpp"

using ghost_v5_config::ghost_brake_mode;
using ghost_v5_config::ghost_encoder_unit;
using ghost_v5_config::ghost_gearset;
using ghost_v5_config::MotorConfigStruct;
namespace ghost_control {

MotorController::MotorController(const MotorConfigStruct &config) :
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
	des_pos_encoder_{0},
	des_vel_rpm_{0.0},
	des_voltage_norm_{0.0},
	des_torque_nm_{0.0},
	cmd_voltage_mv_{0.0},
	position_active_{false},
	velocity_active_{false},
	torque_active_{false},
	voltage_active_{false},
	config_{config}{
}

float MotorController::updateMotor(float position, float velocity){
	// Update Low Pass Filter with velocity measurement
	curr_vel_rpm_ = velocity_filter_.updateFilter(velocity);

	// Update DC Motor Model
	motor_model_.setMotorSpeedRPM(curr_vel_rpm_);

	float max_voltage_mv = config_.motor__max_voltage * 1000;

	// Calculate control inputs
	float position_feedback = (position_active_) ? (des_pos_encoder_ - position) * config_.ctl__pos_gain : 0.0;
	float velocity_feedforward = (velocity_active_) ? motor_model_.getVoltageFromVelocityMillivolts(des_vel_rpm_) * config_.ctl__ff_vel_gain : 0.0;
	float velocity_feedback = (velocity_active_) ? (des_vel_rpm_ - curr_vel_rpm_) * config_.ctl__vel_gain : 0.0;
	float torque_feedforward = (torque_active_) ? motor_model_.getVoltageFromTorqueMillivolts(des_torque_nm_) * config_.ctl__ff_torque_gain : 0.0;
	float voltage_feedforward = (voltage_active_) ? des_voltage_norm_ * max_voltage_mv : 0.0;

	// Set voltage command from sum of controller terms
	cmd_voltage_mv_ = voltage_feedforward + torque_feedforward + velocity_feedforward + velocity_feedback + position_feedback;

	// Clamp actuator limits
	cmd_voltage_mv_ = std::max(max_voltage_mv, std::min(-max_voltage_mv, cmd_voltage_mv_));

	// Motor control mode must be constantly refreshed
	position_active_ = false;
	velocity_active_ = false;
	torque_active_ = false;
	voltage_active_ = false;

	return cmd_voltage_mv_;
}

} // namespace ghost_control