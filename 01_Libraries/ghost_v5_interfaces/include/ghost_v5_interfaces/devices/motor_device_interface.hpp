#pragma once

#include <cstring>
#include "ghost_control/models/dc_motor_model.hpp"
#include "ghost_control/motor_controller.hpp"
#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"

using ghost_util::packByte;
using ghost_util::unpackByte;

using ghost_control::DCMotorModel;
using ghost_control::MotorController;

namespace ghost_v5_interfaces {

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

class MotorDeviceConfig : public DeviceConfig {
public:

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<MotorDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const MotorDeviceConfig *d_rhs = dynamic_cast<const MotorDeviceConfig *>(&rhs);
		return (d_rhs != nullptr) && (port == d_rhs->port) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (reversed == d_rhs->reversed) && (gearset == d_rhs->gearset) && (brake_mode == d_rhs->brake_mode) &&
		       (encoder_units == d_rhs->encoder_units) && (controller_config == d_rhs->controller_config) &&
		       (filter_config == d_rhs->filter_config) && (model_config == d_rhs->model_config);
	}

	bool reversed = false;
	MotorController::Config controller_config;
	SecondOrderLowPassFilter::Config filter_config;
	DCMotorModel::Config model_config;

	// These three map 1:1 to their PROS counterpart on the V5 Side.
	ghost_encoder_unit encoder_units{ghost_encoder_unit::ENCODER_COUNTS};
	ghost_gearset gearset{ghost_gearset::GEARSET_200};
	ghost_brake_mode brake_mode{ghost_brake_mode::BRAKE_MODE_COAST};
};

class MotorDeviceData : public DeviceData {
public:

	// Msg Size
	const int actuator_msg_byte_count = 5 * 4 + 1;
	const int sensor_msg_byte_count = 7 * 4;

	// Actuator Values
	float desired_position = 0.0;   // Degrees
	float desired_velocity = 0.0;   // RPM
	float desired_torque = 0.0;     // N - m
	float desired_voltage = 0.0;    // Normalized (-1.0->1.0)
	float current_limit = 0.0;      // milliAmps
	bool position_control = false;
	bool velocity_control = false;
	bool torque_control = false;
	bool voltage_control = false;

	// Sensor Values
	float curr_position = 0.0;      // Degrees
	float curr_velocity_rpm = 0.0;  // RPM
	float curr_torque_nm = 0.0;     // N - m
	float curr_voltage_mv = 0.0;    // MilliVolts
	float curr_current_ma = 0.0;    // MilliAmps
	float curr_power_w = 0.0;       // Watts
	float curr_temp_c = 0.0;        // Celsius

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<MotorDeviceData>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const MotorDeviceData *d_rhs = dynamic_cast<const MotorDeviceData *>(&rhs);
		return (d_rhs != nullptr) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (desired_position == d_rhs->desired_position) && (desired_velocity == d_rhs->desired_velocity) &&
		       (desired_torque == d_rhs->desired_torque) && (desired_voltage == d_rhs->desired_voltage) &&
		       (current_limit == d_rhs->current_limit) && (position_control == d_rhs->position_control) &&
		       (velocity_control == d_rhs->velocity_control) && (torque_control == d_rhs->torque_control) &&
		       (voltage_control == d_rhs->voltage_control) && (curr_position == d_rhs->curr_position) &&
		       (curr_velocity_rpm == d_rhs->curr_velocity_rpm) && (curr_torque_nm == d_rhs->curr_torque_nm) &&
		       (curr_voltage_mv == d_rhs->curr_voltage_mv) && (curr_current_ma == d_rhs->curr_current_ma) &&
		       (curr_power_w == d_rhs->curr_power_w) && (curr_temp_c == d_rhs->curr_temp_c);
	}

	void checkMsgSize(std::vector<unsigned char> data, int msg_size){
		if(data.size() != msg_size){
			throw std::runtime_error("[MotorDeviceData::checkMsgSize] Error: Motor " + name + " recieved incorrect serial msg size. " +
			                         "Expecting " + std::to_string(msg_size) + " bytes, received " + std::to_string(data.size()) + " bytes.");
		}
	}

	std::vector<unsigned char> serialize(bool to_v5) const override {
		std::vector<unsigned char> msg;
		if(to_v5){
			msg.resize(actuator_msg_byte_count, 0);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(msg_buffer + byte_offset, &desired_position, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &desired_velocity, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &desired_torque, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &desired_voltage, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &current_limit, 4);
			byte_offset += 4;

			unsigned char ctrl_byte = packByte(
				std::vector<bool>{
					position_control,
					velocity_control,
					torque_control,
					voltage_control,
					0,
					0,
					0,
					0});

			memcpy(msg_buffer + byte_offset, &ctrl_byte, 1);
		}
		else{
			msg.resize(sensor_msg_byte_count, 0);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(msg_buffer + byte_offset, &curr_position, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_velocity_rpm, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_torque_nm, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_voltage_mv, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_current_ma, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_power_w, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_temp_c, 4);
		}
		return msg;
	}

	void deserialize(const std::vector<unsigned char>& msg, bool from_coprocessor) override {
		if(from_coprocessor){
			// Actuator Msg
			checkMsgSize(msg, actuator_msg_byte_count);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(&desired_position, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&desired_velocity, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&desired_torque, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&desired_voltage, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&current_limit, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			unsigned char ctrl_byte;
			memcpy(&ctrl_byte, msg_buffer + byte_offset, 1);

			auto ctrl_vec = unpackByte(ctrl_byte);

			position_control = ctrl_vec[0];
			velocity_control = ctrl_vec[1];
			torque_control = ctrl_vec[2];
			voltage_control = ctrl_vec[3];
		}
		else{
			// Sensor Msg
			checkMsgSize(msg, sensor_msg_byte_count);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(&curr_position, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_velocity_rpm, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_torque_nm, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_voltage_mv, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_current_ma, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_power_w, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_temp_c, msg_buffer + byte_offset, 4);
		}
	}
};

} // namespace ghost_v5_interfaces