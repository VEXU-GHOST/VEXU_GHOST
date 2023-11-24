#pragma once

#include <stdint.h>
#include "ghost_v5_core/filters/second_order_low_pass_filter.hpp"
#include "ghost_v5_core/motor/dc_motor_model.hpp"

namespace ghost_v5_core {

class MotorController {
public:
	struct Config {
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

		bool operator==(const Config& rhs) const {
			return (pos_gain == rhs.pos_gain) && (vel_gain == rhs.vel_gain) &&
			       (ff_vel_gain == rhs.ff_vel_gain) && (ff_torque_gain == rhs.ff_torque_gain);
		}
	};

	MotorController(const MotorController::Config &controller_config,
	                const SecondOrderLowPassFilter::Config &filter_config,
	                const DCMotorModel::Config &model_config);

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
	// Configuration
	MotorController::Config controller_config_;
	SecondOrderLowPassFilter::Config filter_config_;
	DCMotorModel::Config model_config_;

	// Velocity Filter
	ghost_v5_core::SecondOrderLowPassFilter velocity_filter_;

	// Motor Model
	ghost_v5_core::DCMotorModel motor_model_;

	// Motor Controller
	int32_t des_pos_encoder_ = 0;
	float des_vel_rpm_ = 0.0;
	float des_torque_nm_ = 0.0;
	float des_voltage_norm_ = 0.0;
	float cmd_voltage_mv_ = 0.0;
	bool position_active_ = false;
	bool velocity_active_ = false;
	bool voltage_active_ = false;
	bool torque_active_ = false;
};

} // namespace ghost_v5_core