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
			device_pair_name_map_[key] = pair;
			device_pair_port_map_[pair.config_ptr->port] = pair;
		}
	}
}


std::vector<unsigned char> RobotHardwareInterface::serialize() const {
	std::vector<unsigned char> serial_data;
	return serial_data;
}
void RobotHardwareInterface::deserialize(std::vector<unsigned char>& msg){
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