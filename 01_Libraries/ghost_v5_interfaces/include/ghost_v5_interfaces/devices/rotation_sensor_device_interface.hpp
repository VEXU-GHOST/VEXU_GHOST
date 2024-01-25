#pragma once

#include <cstring>
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"

using ghost_util::getBit;
using ghost_util::packByte;
using ghost_util::setBit;

namespace ghost_v5_interfaces {

namespace devices {

class RotationSensorDeviceData : public DeviceData {
public:

	struct SerialConfig {
		SerialConfig(){
		}

		bool operator==(const SerialConfig &rhs) const {
			return (send_angle_data == rhs.send_angle_data) &&
			       (send_position_data == rhs.send_position_data) &&
			       (send_velocity_data == rhs.send_velocity_data);
		}

		bool send_angle_data = true;
		bool send_position_data = true;
		bool send_velocity_data = true;
	};

	RotationSensorDeviceData(std::string name, SerialConfig serial_config = SerialConfig()) :
		DeviceData(name, device_type_e::ROTATION_SENSOR),
		serial_config_(serial_config){
	}

	// Msg Size
	int getActuatorPacketSize() const {
		return 0;
	}

	int getSensorPacketSize() const {
		int packet_size = 0;
		packet_size += 4 * ((int) serial_config_.send_angle_data);
		packet_size += 4 * ((int) serial_config_.send_position_data);
		packet_size += 4 * ((int) serial_config_.send_velocity_data);
		return packet_size;
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
		       (position == d_rhs->position) && (velocity == d_rhs->velocity) && (angle == d_rhs->angle) &&
		       (serial_config_ == d_rhs->serial_config_);
	}

	std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override {
		std::vector<unsigned char> msg;
		if(hardware_type == hardware_type_e::V5_BRAIN){
			msg.resize(getSensorPacketSize(), 0);
			auto msg_buffer = msg.data();
			int byte_offset = 0;

			if(serial_config_.send_angle_data){
				memcpy(msg_buffer + byte_offset, &angle, 4);
				byte_offset += 4;
			}
			if(serial_config_.send_position_data){
				memcpy(msg_buffer + byte_offset, &position, 4);
				byte_offset += 4;
			}
			if(serial_config_.send_velocity_data){
				memcpy(msg_buffer + byte_offset, &velocity, 4);
				byte_offset += 4;
			}
			checkMsgSize(byte_offset, getSensorPacketSize());
		}
		return msg;
	}

	void deserialize(const std::vector<unsigned char>& msg, hardware_type_e hardware_type) override {
		if(hardware_type == hardware_type_e::COPROCESSOR){
			// Sensor Msg
			checkMsgSize(msg, getSensorPacketSize());
			auto msg_buffer = msg.data();
			int byte_offset = 0;
			if(serial_config_.send_angle_data){
				memcpy(&angle, msg_buffer + byte_offset, 4);
				byte_offset += 4;
			}
			if(serial_config_.send_position_data){
				memcpy(&position, msg_buffer + byte_offset, 4);
				byte_offset += 4;
			}
			if(serial_config_.send_velocity_data){
				memcpy(&velocity, msg_buffer + byte_offset, 4);
				byte_offset += 4;
			}
		}
	}

	SerialConfig serial_config_;
};

class RotationSensorDeviceConfig : public DeviceConfig {
public:

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<RotationSensorDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const RotationSensorDeviceConfig *d_rhs = dynamic_cast<const RotationSensorDeviceConfig *>(&rhs);
		return (d_rhs != nullptr) && (port == d_rhs->port) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (reversed == d_rhs->reversed) && (data_rate == d_rhs->data_rate) &&
		       (serial_config == d_rhs->serial_config);
	}

	bool reversed = false;
	uint32_t data_rate = 5;
	RotationSensorDeviceData::SerialConfig serial_config;
};

} // namespace devices

} // namespace ghost_v5_interfaces