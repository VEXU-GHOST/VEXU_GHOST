#pragma once

#include <cstring>
#include "ghost_util/byte_utils.hpp"
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"

using ghost_util::packByte;
using ghost_util::unpackByte;

namespace ghost_v5_interfaces {

namespace devices {

class InertialSensorDeviceData : public DeviceData {
public:

	struct SerialConfig {
		SerialConfig(){
		}

		bool operator==(const SerialConfig &rhs) const {
			return (send_accel_data == rhs.send_accel_data) && (send_gyro_data == rhs.send_gyro_data) &&
			       (send_heading_data == rhs.send_heading_data);
		}

		// Sensor Update Msg Config
		bool send_accel_data = true;
		bool send_gyro_data = true;
		bool send_heading_data = true;
	};

	InertialSensorDeviceData(std::string name, SerialConfig serial_config = SerialConfig()) :
		DeviceData(name, device_type_e::INERTIAL_SENSOR),
		serial_config_(serial_config){
	}

	int getActuatorPacketSize() const override {
		return 0;
	}

	int getSensorPacketSize() const override {
		int packet_size = 0;
		packet_size += 4 * 3 * ((int) serial_config_.send_accel_data);
		packet_size += 4 * 3 * ((int) serial_config_.send_gyro_data);
		packet_size += 4 * ((int) serial_config_.send_heading_data);
		return packet_size;
	}

	// Inertial Sensor State
	float x_accel = 0.0;
	float y_accel = 0.0;
	float z_accel = 0.0;
	float x_rate = 0.0;
	float y_rate = 0.0;
	float z_rate = 0.0;
	float heading = 0.0;

	// Serial Config
	SerialConfig serial_config_;

	void update(std::shared_ptr<DeviceData> data_ptr) override {
		auto inertial_data_ptr = data_ptr->as<InertialSensorDeviceData>();
		x_accel = inertial_data_ptr->x_accel;
		y_accel = inertial_data_ptr->y_accel;
		z_accel = inertial_data_ptr->z_accel;
		x_rate = inertial_data_ptr->x_rate;
		y_rate = inertial_data_ptr->y_rate;
		z_rate = inertial_data_ptr->z_rate;
		heading = inertial_data_ptr->heading;
	}

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<InertialSensorDeviceData>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const InertialSensorDeviceData *d_rhs = dynamic_cast<const InertialSensorDeviceData *>(&rhs);
		return (d_rhs != nullptr) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (x_rate == d_rhs->x_rate) && (y_rate == d_rhs->y_rate) && (z_rate == d_rhs->z_rate) &&
		       (x_accel == d_rhs->x_accel) && (y_accel == d_rhs->y_accel) && (z_accel == d_rhs->z_accel) &&
		       (heading == d_rhs->heading);
	}

	std::vector<unsigned char> serialize(hardware_type_e hardware_type) const override {
		std::vector<unsigned char> msg;
		int byte_offset = 0;
		if((hardware_type == hardware_type_e::V5_BRAIN)){
			msg.resize(getSensorPacketSize(), 0);
			auto msg_buffer = msg.data();
			int byte_offset = 0;

			if(serial_config_.send_accel_data){
				memcpy(msg_buffer + byte_offset, &x_accel, 4);
				byte_offset += 4;
				memcpy(msg_buffer + byte_offset, &y_accel, 4);
				byte_offset += 4;
				memcpy(msg_buffer + byte_offset, &z_accel, 4);
				byte_offset += 4;
			}

			if(serial_config_.send_gyro_data){
				memcpy(msg_buffer + byte_offset, &x_rate, 4);
				byte_offset += 4;
				memcpy(msg_buffer + byte_offset, &y_rate, 4);
				byte_offset += 4;
				memcpy(msg_buffer + byte_offset, &z_rate, 4);
				byte_offset += 4;
			}

			if(serial_config_.send_heading_data){
				memcpy(msg_buffer + byte_offset, &heading, 4);
				byte_offset += 4;
			}
		}

		int msg_size = (hardware_type == hardware_type_e::V5_BRAIN) ? getSensorPacketSize() : getActuatorPacketSize();
		checkMsgSize(msg, msg_size);
		return msg;
	}

	void deserialize(const std::vector<unsigned char>& msg, hardware_type_e hardware_type) override {
		int msg_size = (hardware_type == hardware_type_e::V5_BRAIN) ? getActuatorPacketSize() : getSensorPacketSize();
		checkMsgSize(msg, msg_size);

		if(hardware_type == hardware_type_e::COPROCESSOR){
			checkMsgSize(msg, getSensorPacketSize());
			auto msg_buffer = msg.data();
			int byte_offset = 0;

			if(serial_config_.send_accel_data){
				memcpy(&x_accel, msg_buffer + byte_offset,  4);
				byte_offset += 4;
				memcpy(&y_accel, msg_buffer + byte_offset,  4);
				byte_offset += 4;
				memcpy(&z_accel, msg_buffer + byte_offset,  4);
				byte_offset += 4;
			}

			if(serial_config_.send_gyro_data){
				memcpy(&x_rate, msg_buffer + byte_offset, 4);
				byte_offset += 4;
				memcpy(&y_rate, msg_buffer + byte_offset, 4);
				byte_offset += 4;
				memcpy(&z_rate, msg_buffer + byte_offset, 4);
				byte_offset += 4;
			}

			if(serial_config_.send_heading_data){
				memcpy(&heading, msg_buffer + byte_offset,  4);
				byte_offset += 4;
			}
		}
	}
};

class InertialSensorDeviceConfig : public DeviceConfig {
public:
	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<InertialSensorDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const InertialSensorDeviceConfig *d_rhs = dynamic_cast<const InertialSensorDeviceConfig *>(&rhs);
		return (d_rhs != nullptr) && (port == d_rhs->port) && (name == d_rhs->name) && (type == d_rhs->type) &&
		       (serial_config == d_rhs->serial_config);
		;
	}
	InertialSensorDeviceData::SerialConfig serial_config;
};

} // namespace devices

} // namespace ghost_v5_interfaces