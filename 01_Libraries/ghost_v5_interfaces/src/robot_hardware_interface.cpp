#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"

#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

namespace ghost_v5_interfaces {

RobotHardwareInterface::RobotHardwareInterface(std::shared_ptr<DeviceConfigMap> robot_config_ptr,
                                               hardware_type_e hardware_type) :
	robot_config_ptr_(robot_config_ptr->clone()),
	hardware_type_(hardware_type){
	{
		for(const auto & [key, val] : *robot_config_ptr_){
			DevicePair pair;
			pair.config_ptr = val;
			if(pair.config_ptr->type == device_type_e::MOTOR){
				pair.data_ptr = std::make_shared<MotorDeviceData>();
			}
			else if(pair.config_ptr->type == device_type_e::ROTATION_SENSOR){
				pair.data_ptr = std::make_shared<RotationSensorDeviceData>();
			}
			else{
				throw std::runtime_error("[RobotHardwareInterface::RobotHardwareInterface()] Device type " + std::to_string(pair.config_ptr->type) + " is unsupported!");
			}
			pair.data_ptr->name = val->name;
			device_pair_name_map_[key] = pair;
			device_pair_port_map_[pair.config_ptr->port] = pair;
		}

		digital_io_vector_.resize(8);
	}

	use_secondary_joystick_ = robot_config_ptr_->use_secondary_joystick;
	primary_joystick_data_ptr_ = std::make_shared<JoystickDeviceData>();
	primary_joystick_data_ptr_->name = "primary_joystick";
	secondary_joystick_data_ptr_ = std::make_shared<JoystickDeviceData>();
	secondary_joystick_data_ptr_->name = "secondary_joystick";
}


std::vector<unsigned char> RobotHardwareInterface::serialize() const {
	std::vector<unsigned char> serial_data;

	// Send state of all Digital IO Ports
	serial_data.push_back(packByte(digital_io_vector_));
	bool coprocessor_to_v5 = (hardware_type_ == hardware_type_e::COPROCESSOR);
	bool v5_to_coprocessor = !coprocessor_to_v5;

	// Only send competition state and joystick info from V5 Brain to Coprocessor
	if(v5_to_coprocessor){
		serial_data.push_back(packByte(std::vector<bool>{
				is_disabled_, is_autonomous_, is_connected_, 0, 0, 0, 0, 0
			}));
	}

	auto j1_serial_msg = primary_joystick_data_ptr_->serialize(coprocessor_to_v5);
	serial_data.insert(serial_data.end(), j1_serial_msg.begin(), j1_serial_msg.end());

	if(use_secondary_joystick_){
		auto j2_serial_msg = secondary_joystick_data_ptr_->serialize(coprocessor_to_v5);
		serial_data.insert(serial_data.end(), j2_serial_msg.begin(), j2_serial_msg.end());
	}

	for(const auto & [key, val] : device_pair_port_map_){
		auto device_serial_msg = val.data_ptr->serialize(coprocessor_to_v5);
		serial_data.insert(serial_data.end(), device_serial_msg.begin(), device_serial_msg.end());
	}
	return serial_data;
}
void RobotHardwareInterface::deserialize(std::vector<unsigned char>& msg){
	bool coprocessor_to_v5 = (hardware_type_ == hardware_type_e::V5_BRAIN);
	bool v5_to_coprocessor = !coprocessor_to_v5;
	int byte_offset = 0;
	// Unpack Digital IO
	digital_io_vector_ = unpackByte(msg[byte_offset]);
	byte_offset++;

	if(v5_to_coprocessor){
		// Unpack competition state
		auto packet_start_byte = unpackByte(msg[byte_offset]);
		is_disabled_ = packet_start_byte[0];
		is_autonomous_ = packet_start_byte[1];
		is_connected_ = packet_start_byte[2];
		byte_offset++;
	}

	// Unpack Primary joystick
	int joy_msg_len;
	if(hardware_type_ == hardware_type_e::COPROCESSOR){
		joy_msg_len = primary_joystick_data_ptr_->getSensorPacketSize();
	}
	else if(hardware_type_ == hardware_type_e::V5_BRAIN){
		joy_msg_len = primary_joystick_data_ptr_->getActuatorPacketSize();
	}
	else{
		throw std::runtime_error("[RobotHardwareInterface::deserialize] Error: Attempted to deserialize with unsupported hardware type.");
	}

	// Unpack joystick data
	if(joy_msg_len != 0){
		auto start_itr_j1 = msg.begin() + byte_offset;
		primary_joystick_data_ptr_->deserialize(std::vector<unsigned char>(start_itr_j1, start_itr_j1 + joy_msg_len), v5_to_coprocessor);
		byte_offset += joy_msg_len;

		if(use_secondary_joystick_){
			auto start_itr_j2 = msg.begin() + byte_offset;
			secondary_joystick_data_ptr_->deserialize(std::vector<unsigned char>(start_itr_j2, start_itr_j2 + joy_msg_len), v5_to_coprocessor);
			byte_offset += joy_msg_len;
		}
	}

	// Unpack each device in device tree
	for(const auto & [key, val] : device_pair_port_map_){
		int msg_len;
		if(hardware_type_ == hardware_type_e::COPROCESSOR){
			msg_len = val.data_ptr->getSensorPacketSize();
		}
		else if(hardware_type_ == hardware_type_e::V5_BRAIN){
			msg_len = val.data_ptr->getActuatorPacketSize();
		}
		else{
			throw std::runtime_error("[RobotHardwareInterface::deserialize] Error: Attempted to deserialize with unsupported hardware type.");
		}

		auto start_itr = msg.begin() + byte_offset;
		val.data_ptr->deserialize(std::vector<unsigned char>(start_itr, start_itr + msg_len), coprocessor_to_v5);
		byte_offset += msg_len;
	}
}

bool RobotHardwareInterface::operator==(const RobotHardwareInterface& rhs) const {
	bool eq = (hardware_type_ == rhs.hardware_type_);
	for(const auto& [key, val] : device_pair_name_map_){
		if(rhs.device_pair_name_map_.count(key) == 0){
			return false;
		}
		auto rhs_device_pair = rhs.device_pair_name_map_.at(key);
		eq &= (*val.data_ptr == *rhs_device_pair.data_ptr);
		eq &= (*val.config_ptr == *rhs_device_pair.config_ptr);
	}

	for(const auto& [key, val] : device_pair_port_map_){
		if(rhs.device_pair_port_map_.count(key) == 0){
			return false;
		}
	}

	return eq;
}

} // namespace ghost_v5_interfaces