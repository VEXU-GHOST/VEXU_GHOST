#pragma once

#include <map>
#include <memory>
#include <unordered_map>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"

namespace ghost_v5_interfaces {

enum competition_state_e {
	DISABLED,
	AUTONOMOUS,
	TELEOP
};

enum hardware_type_e {
	COPROCESSOR,
	V5_BRAIN
};

struct DevicePair {
	std::shared_ptr<const DeviceConfig> config_ptr;
	std::shared_ptr<DeviceData> data_ptr;
};

class RobotHardwareInterface {
public:
	RobotHardwareInterface(std::shared_ptr<DeviceConfigMap> robot_config_ptr, hardware_type_e hardware_type);

	DevicePair getDevicePair(std::string name){
		throwOnNonexistentDevice(name);
		return device_pair_name_map_.at(name);
	}

	std::shared_ptr<const DeviceConfig> getDeviceConfig(std::string name){
		throwOnNonexistentDevice(name);
		return device_pair_name_map_.at(name).config_ptr;
	}

	std::shared_ptr<DeviceData> getDeviceData(std::string name){
		throwOnNonexistentDevice(name);
		return device_pair_name_map_.at(name).data_ptr;
	}

	void setDeviceData(std::string name, std::shared_ptr<DeviceData> device_data){
		throwOnNonexistentDevice(name);
		device_pair_name_map_.at(name).data_ptr->update(device_data);
	}

	const competition_state_e& getCompetitionState(){
		return competition_state_;
	}

	void setCompetitionState(const competition_state_e& state){
		competition_state_ = state;
	}

	std::shared_ptr<JoystickDeviceData> getJoystickData(){
		return joystick_data_ptr_->clone()->as<JoystickDeviceData>();
	}

	void setJoystickData(std::shared_ptr<JoystickDeviceData>& data){
		joystick_data_ptr_ = data->clone()->as<JoystickDeviceData>();
	}

	std::map<int, DevicePair>::iterator begin(){
		return device_pair_port_map_.begin();
	}

	std::map<int, DevicePair>::iterator end(){
		return device_pair_port_map_.end();
	}

	std::vector<unsigned char> serialize() const;
	void deserialize(std::vector<unsigned char>& msg);

	bool operator==(const RobotHardwareInterface& rhs) const;

private:

	void throwOnNonexistentDevice(std::string name){
		if(device_pair_name_map_.count(name) == 0){
			throw std::runtime_error("[RobotHardwareInterface::getDeviceConfig] Error: Device name " + name + " does not exist!");
		}
	}

	competition_state_e competition_state_;
	std::shared_ptr<JoystickDeviceData> joystick_data_ptr_;

	std::unordered_map<std::string, DevicePair> device_pair_name_map_;
	std::map<int, DevicePair> device_pair_port_map_;
	std::shared_ptr<DeviceConfigMap> robot_config_ptr_;
	hardware_type_e hardware_type_;
};

} // namespace ghost_v5_interfaces