#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"

#include "ghost_v5_interfaces/robot_hardware_interface.hpp"


using namespace ghost_v5_interfaces::devices;
using namespace ghost_v5_interfaces;
namespace ghost_v5_interfaces {

RobotHardwareInterface::RobotHardwareInterface(std::shared_ptr<DeviceConfigMap> robot_config_ptr,
                                               hardware_type_e hardware_type) :
	robot_config_ptr_(robot_config_ptr->clone()),
	hardware_type_(hardware_type){
	sensor_update_msg_length_ = 0;
	actuator_command_msg_length_ = 0;

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
		device_names_ordered_by_port_.push_back(val->name);

		// Update msg lengths based on each device
		sensor_update_msg_length_ += pair.data_ptr->getSensorPacketSize();
		actuator_command_msg_length_ += pair.data_ptr->getActuatorPacketSize();
	}

	// Add Digital IO to actuator command msg
	actuator_command_msg_length_ += 1;
	digital_io_vector_.resize(8);

	use_secondary_joystick_ = robot_config_ptr_->use_secondary_joystick;
	primary_joystick_data_ptr_ = std::make_shared<JoystickDeviceData>();
	primary_joystick_data_ptr_->name = "primary_joystick";
	secondary_joystick_data_ptr_ = std::make_shared<JoystickDeviceData>();
	secondary_joystick_data_ptr_->name = "secondary_joystick";

	// Add Competition State and joysticks to sensor update msg
	sensor_update_msg_length_ += 1;
	sensor_update_msg_length_ += primary_joystick_data_ptr_->getSensorPacketSize();
	sensor_update_msg_length_ += (use_secondary_joystick_) ? secondary_joystick_data_ptr_->getSensorPacketSize() : 0;
}

std::vector<unsigned char> RobotHardwareInterface::serialize() const {
	std::vector<unsigned char> serial_data;
	std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);

	// Only send competition state and joystick info from V5 Brain to Coprocessor
	if(hardware_type_ == hardware_type_e::V5_BRAIN){
		serial_data.push_back(packByte(std::vector<bool>{
				is_disabled_, is_autonomous_, is_connected_, 0, 0, 0, 0, 0
			}));
	}
	else if(hardware_type_ == hardware_type_e::COPROCESSOR){
		// Send state of all Digital IO Ports
		serial_data.push_back(packByte(digital_io_vector_));
	}

	auto j1_serial_msg = primary_joystick_data_ptr_->serialize(hardware_type_);
	serial_data.insert(serial_data.end(), j1_serial_msg.begin(), j1_serial_msg.end());

	if(use_secondary_joystick_){
		auto j2_serial_msg = secondary_joystick_data_ptr_->serialize(hardware_type_);
		serial_data.insert(serial_data.end(), j2_serial_msg.begin(), j2_serial_msg.end());
	}

	for(const auto & [key, val] : device_pair_port_map_){
		auto device_serial_msg = val.data_ptr->serialize(hardware_type_);
		serial_data.insert(serial_data.end(), device_serial_msg.begin(), device_serial_msg.end());
	}
	return serial_data;
}

void RobotHardwareInterface::deserialize(std::vector<unsigned char>& msg){
	int byte_offset = 0;
	std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);

	if(hardware_type_ == hardware_type_e::V5_BRAIN){
		// Unpack Digital IO
		digital_io_vector_ = unpackByte(msg[byte_offset]);
		byte_offset++;
	}

	if(hardware_type_ == hardware_type_e::COPROCESSOR){
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
		primary_joystick_data_ptr_->deserialize(std::vector<unsigned char>(start_itr_j1, start_itr_j1 + joy_msg_len), hardware_type_);
		byte_offset += joy_msg_len;

		if(use_secondary_joystick_){
			auto start_itr_j2 = msg.begin() + byte_offset;
			secondary_joystick_data_ptr_->deserialize(std::vector<unsigned char>(start_itr_j2, start_itr_j2 + joy_msg_len), hardware_type_);
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
		val.data_ptr->deserialize(std::vector<unsigned char>(start_itr, start_itr + msg_len), hardware_type_);
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

DevicePair RobotHardwareInterface::getDevicePair(const std::string& name){
	throwOnNonexistentDevice(name);
	return device_pair_name_map_.at(name).clone();
}

std::shared_ptr<const DeviceConfig> RobotHardwareInterface::getDeviceConfig(const std::string& name){
	throwOnNonexistentDevice(name);
	return device_pair_name_map_.at(name).config_ptr;
}

std::shared_ptr<DeviceData> RobotHardwareInterface::getDeviceData(const std::string& name){
	throwOnNonexistentDevice(name);
	return device_pair_name_map_.at(name).data_ptr->clone()->as<DeviceData>();
}

std::shared_ptr<DeviceData> RobotHardwareInterface::getDeviceData(int port){
	if(device_pair_port_map_.count(port) == 0){
		throw std::runtime_error("[RobotHardwareInterface::getDeviceConfig] Error: Device port " + std::to_string(port) + " is not in use!");
	}
	return device_pair_port_map_.at(port).data_ptr->clone()->as<DeviceData>();
}

void RobotHardwareInterface::setDeviceData(std::shared_ptr<DeviceData> device_data){
	throwOnNonexistentDevice(device_data->name);

	std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
	device_pair_name_map_.at(device_data->name).data_ptr->update(device_data);
}

void RobotHardwareInterface::setDeviceData(std::string name, std::shared_ptr<DeviceData> device_data){
	throwOnNonexistentDevice(name);
	device_data->name = name;

	std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
	device_pair_name_map_.at(name).data_ptr->update(device_data);
}

std::shared_ptr<JoystickDeviceData> RobotHardwareInterface::getPrimaryJoystickData(){
	return primary_joystick_data_ptr_->clone()->as<JoystickDeviceData>();
}

std::shared_ptr<JoystickDeviceData> RobotHardwareInterface::getSecondaryJoystickData(){
	return secondary_joystick_data_ptr_->clone()->as<JoystickDeviceData>();
}

void RobotHardwareInterface::setPrimaryJoystickData(std::shared_ptr<JoystickDeviceData>& data_ptr){
	std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
	primary_joystick_data_ptr_->update(data_ptr);
}


void RobotHardwareInterface::setSecondaryJoystickData(std::shared_ptr<JoystickDeviceData>& data_ptr){
	if(use_secondary_joystick_){
		std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
		secondary_joystick_data_ptr_->update(data_ptr);
	}
	else{
		throw std::runtime_error("[RobotHardwareInterface] Error: Robot is not configured to use secondary joystick");
	}
}

void RobotHardwareInterface::setDigitalIOPorts(const std::vector<bool>& digital_io_vector){
	std::unique_lock<CROSSPLATFORM_MUTEX_T> update_lock(update_mutex_);
	digital_io_vector_ = digital_io_vector;
}

const std::vector<bool>& RobotHardwareInterface::getDigitalIOPorts(){
	return digital_io_vector_;
}

void RobotHardwareInterface::throwOnNonexistentDevice(const std::string& name){
	if(device_pair_name_map_.count(name) == 0){
		throw std::runtime_error("[RobotHardwareInterface::getDeviceConfig] Error: Device name " + name + " does not exist!");
	}
}

} // namespace ghost_v5_interfaces