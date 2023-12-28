#pragma once

#include <cstring>
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"

using ghost_util::getBit;
using ghost_util::packByte;
using ghost_util::setBit;

namespace ghost_v5_interfaces {

namespace devices {

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

	RotationSensorDeviceData(){
		type = device_type_e::ROTATION_SENSOR;
	}

	// Msg Size
	int getActuatorPacketSize() const {
		return 0;
	}

	int getSensorPacketSize() const {
		return 4 * 3;
	}

	// Sensor Values
	float angle = 0.0;
	float position = 0.0;
	float velocity = 0.0;

	virtual void update(std::shared_ptr<DeviceData> data_ptr){
		auto rotation_sensor_data_ptr = data_ptr->as<RotationSensorDeviceData>();
		angle = rotation_sensor_data_ptr->angle;
		position = rotation_sensor_data_ptr->position;
		velocity = rotation_sensor_data_ptr->velocity;
	}

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<RotationSensorDeviceData>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const RotationSensorDeviceData *d_rhs = dynamic_cast<const RotationSensorDeviceData *>(&rhs);
		return (d_rhs != nullptr) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (position == d_rhs->position) && (velocity == d_rhs->velocity) && (angle == d_rhs->angle);
	}

	std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override {
		std::vector<unsigned char> msg;
		if(hardware_type == hardware_type_e::V5_BRAIN){
			msg.resize(getSensorPacketSize(), 0);
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(msg_buffer + byte_offset, &angle, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &position, 4);
			byte_offset += 4;
			memcpy(msg_buffer + byte_offset, &velocity, 4);
		}
		return msg;
	}

	void deserialize(const std::vector<unsigned char>& msg, hardware_type_e hardware_type) override {
		if(hardware_type == hardware_type_e::COPROCESSOR){
			// Sensor Msg
			checkMsgSize(msg, getSensorPacketSize());
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			memcpy(&angle, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&position, msg_buffer + byte_offset, 4);
			byte_offset += 4;
			memcpy(&velocity, msg_buffer + byte_offset, 4);
			byte_offset += 4;
		}
	}
};

} // namespace devices

} // namespace ghost_v5_interfaces