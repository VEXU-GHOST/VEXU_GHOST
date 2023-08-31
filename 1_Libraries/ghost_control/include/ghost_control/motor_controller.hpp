#pragma once

#include <map>
#include "ghost_common/v5_robot_config_defs.hpp"
#include "ghost_control/models/dc_motor_model.hpp"
#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"

namespace ghost_control {

class MotorController {
public:
	MotorController(const ghost_v5_config::MotorConfigStruct &config);

	/**
	 * @brief Updates motor with new position and velocity readings, returning a voltage command based on the
	 * current controller setpoints.
	 *
	 * Should be called in a loop at a constant frequency (important for velocity filtering).
	 *
	 * @param position encoder position (should match with encoder unit from Config)
	 * @param velocity encoder velocity (RPM)
	 * @return float cmd_voltage_mv
	 */
	float updateMotor(float position, float velocity);

	/**
	 * @brief Returns last voltage command in millivolts
	 *
	 * @return float cmd_voltage_mv
	 */
	float getVoltageCommand(){
		return cmd_voltage_mv_;
	}

	/**
	 * @brief Returns filtered velocity in RPM
	 *
	 * @return float
	 */
	float getVelocityFilteredRPM(){
		return velocity_filter_.getCurrentState();
	}

	/**
	 * @brief Updates the controller setpoints for the motor. Will not modify the control mode (which setpoints are used).
	 *
	 * @param position  (Encoder Units)
	 * @param velocity  (RPM)
	 * @param voltage   (-1.0 -> 1.0)
	 * @param torque    (Newton-Meters)
	 */
	void setMotorCommand(float position, float velocity, float voltage, float torque = 0.0){
		des_pos_encoder_ = position;
		des_vel_rpm_ = velocity;
		des_voltage_norm_ = voltage;
		des_torque_nm_ = torque;
	}

	/**
	 * @brief Set the control mode for the motor (which setpoints are used to calculate voltage command).
	 *
	 * Torque control is experimental / niche, so not recommended for general use.
	 *
	 * @param position_active
	 * @param velocity_active
	 * @param voltage_active
	 * @param torque_active
	 */
	void setControlMode(bool position_active, bool velocity_active, bool voltage_active, bool torque_active = false){
		position_active_ = position_active;
		velocity_active_ = velocity_active;
		voltage_active_ = voltage_active;
		torque_active_ = torque_active;
	}

	bool positionActive(){
		return position_active_;
	}

	bool velocityActive(){
		return velocity_active_;
	}

	bool voltageActive(){
		return voltage_active_;
	}

	bool torqueActive(){
		return torque_active_;
	}

	bool controllerActive(){
		return position_active_ || velocity_active_ || torque_active_ || voltage_active_;
	}

protected:
	// Motor Config
	ghost_v5_config::MotorConfigStruct config_;

	// Velocity Filtering
	ghost_estimation::SecondOrderLowPassFilter velocity_filter_;
	float curr_vel_rpm_;

	// Motor Controller
	int32_t des_pos_encoder_;
	float des_vel_rpm_;
	float des_torque_nm_;
	float des_voltage_norm_;
	float cmd_voltage_mv_;
	float ctl_rpm_deadband_;
	bool position_active_;
	bool velocity_active_;
	bool voltage_active_;
	bool torque_active_;

	// Motor Models
	ghost_control::DCMotorModel motor_model_;
};

} // namespace ghost_control