#pragma once

#include "ghost_v5_core/device_config.hpp"
#include "ghost_v5_core/filters/second_order_low_pass_filter.hpp"
#include "ghost_v5_core/motor/dc_motor_model.hpp"

namespace ghost_v5_core {

enum ghost_gearset {
	GEARSET_100,
	GEARSET_200,
	GEARSET_600
};

enum ghost_brake_mode {
	BRAKE_MODE_COAST,
	BRAKE_MODE_BRAKE,
	BRAKE_MODE_HOLD,
	BRAKE_MODE_INVALID
};

enum ghost_encoder_unit {
	ENCODER_DEGREES,
	ENCODER_ROTATIONS,
	ENCODER_COUNTS,
	ENCODER_INVALID
};

struct MotorControllerConfig {
	// FF-PD Controller
	// pos_gain and vel_gain are standard PD control
	// ff_vel_gain takes a velocity setpoint and scales it to estimate the required open-loop voltage
	// Ideally this is 1.0. If your motor runs faster than nominal, increase this a bit.
	// If you zero the other gains, you can tune this by sending a velocity and tweaking until true velocity matches.
	// ff_torque_gain does the same as above but for torque. Controlling torque with voltage is not very accurate, fyi.
	float pos_gain{0.0};
	float vel_gain{10.0};
	float ff_vel_gain{1.0};
	float ff_torque_gain{0.0};

	bool operator==(const MotorControllerConfig& rhs) const {
		return (pos_gain == rhs.pos_gain) && (vel_gain == rhs.vel_gain) &&
		       (ff_vel_gain == rhs.ff_vel_gain) && (ff_torque_gain == rhs.ff_torque_gain);
	}
};

struct V5MotorInterfaceConfig {
	// These three map 1:1 to their PROS counterpart on the V5 Side.
	ghost_encoder_unit encoder_units{ghost_encoder_unit::ENCODER_DEGREES};
	ghost_gearset gearset{ghost_gearset::GEARSET_600};
	ghost_brake_mode brake_mode{ghost_brake_mode::BRAKE_MODE_COAST};

	// Configures control and estimation modules for a given motor
	MotorControllerConfig controller;

	// 2nd Order Low Pass Filter for Motor Velocity
	// If you aren't familiar with LPFs, default tuning is probably fine for any VEX system.
	// If you want it smoother, lower the cutoff frequency.
	// "Ideal" cutoff frequency is 10 / ((time for system to go from zero to full speed given max voltage) * 0.632)
	LowPassFilterConfig filter;
	MotorModelConfig model;

	bool operator==(const V5MotorInterfaceConfig& rhs) const {
		return (encoder_units == rhs.encoder_units) && (gearset == rhs.gearset) && (brake_mode == rhs.brake_mode) &&
		       (controller == rhs.controller) && (filter == rhs.filter) && (model == rhs.model);
	}
};

class MotorController {
public:

	MotorController(const V5MotorInterfaceConfig &config);

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
	ghost_v5_core::MotorConfigStruct config_;

	// Velocity Filtering
	ghost_v5_core::SecondOrderLowPassFilter velocity_filter_;
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
	ghost_v5_core::DCMotorModel motor_model_;
};

} // namespace ghost_v5_core