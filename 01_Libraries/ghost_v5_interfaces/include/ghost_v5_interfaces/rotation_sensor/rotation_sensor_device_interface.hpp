#pragma once

#include <cstring>
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/base/device_interfaces.hpp"

using ghost_util::getBit;
using ghost_util::packByte;
using ghost_util::setBit;

namespace ghost_v5_interfaces {

class RotationSensorDeviceConfig : public DeviceConfig {
public:

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<RotationSensorDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const RotationSensorDeviceConfig *d_rhs = dynamic_cast<const RotationSensorDeviceConfig *>(&rhs);
		return (d_rhs != nullptr) && (port == d_rhs->port) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (reversed == d_rhs->reversed) && (data_rate == d_rhs->data_rate);
	}

	bool reversed = false;
	uint32_t data_rate = 5;
};


class RotationSensorDeviceData : public DeviceData {
public:

	// Msg Size
	const int actuator_msg_byte_count = 0;
	const int sensor_msg_byte_count = 2 * 4;

	// Sensor Values
	float curr_position = 0.0;      // Degrees
	float curr_velocity_rpm = 0.0;  // RPM

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<RotationSensorDeviceData>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const RotationSensorDeviceData *d_rhs = dynamic_cast<const RotationSensorDeviceData *>(&rhs);
		return (d_rhs != nullptr) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (curr_position == d_rhs->curr_position) && (curr_velocity_rpm == d_rhs->curr_velocity_rpm);
	}

	void checkMsgSize(std::vector<unsigned char> data, int msg_size){
		if(data.size() != msg_size){
			throw std::runtime_error("[RotationSensorDeviceData::checkMsgSize] Error: Rotation Sensor " + name +
			                         " recieved incorrect serial msg size. " + "Expecting " + std::to_string(msg_size) +
			                         " bytes, received " + std::to_string(data.size()) + " bytes.");
		}
	}

	std::vector<unsigned char> serialize(bool to_v5) const override {
		std::vector<unsigned char> msg;
		if(!to_v5){
			msg.resize(sensor_msg_byte_count, 0);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(msg_buffer + byte_offset, &curr_position, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &curr_velocity_rpm, 4);
		}
		return msg;
	}

	void deserialize(const std::vector<unsigned char>& msg, bool from_coprocessor) override {
		if(!from_coprocessor){
			// Sensor Msg
			checkMsgSize(msg, sensor_msg_byte_count);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(&curr_position, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&curr_velocity_rpm, msg_buffer + byte_offset, 4);
			byte_offset += 4;
		}
	}
};

} // namespace ghost_v5_interfaces