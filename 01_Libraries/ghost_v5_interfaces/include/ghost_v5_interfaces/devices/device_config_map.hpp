#pragma once

#include <iostream>
#include "ghost_v5_interfaces/devices/device_interfaces.hpp"


namespace ghost_v5_interfaces {

class DeviceConfigMap {
public:
	DeviceConfigMap() = default;

	void addDeviceConfig(std::shared_ptr<DeviceConfig> device){
		if(device_configs_.count(device->name) != 0){
			throw std::runtime_error("[DeviceConfigMap::addDeviceConfig] Error: " + device->name + " already exists!");
		}
		else if(port_to_device_name_map_.count(device->port) != 0){
			throw std::runtime_error("[DeviceConfigMap::addDeviceConfig] Error: Port " + std::to_string(device->port) + " is already in use!");
		}
		else{
			device_configs_[device->name] = device->clone()->as<DeviceConfig>();
			port_to_device_name_map_[device->port] = device->name;
		}
	}

	std::shared_ptr<const DeviceConfig> getDeviceConfig(std::string device_name) const {
		if(device_configs_.count(device_name) == 1){
			return device_configs_.at(device_name);
		}
		else{
			throw std::runtime_error("[DeviceConfigMap::getDeviceConfig] Error: Device " + device_name + " does not exist!");
		}
	}

	bool getDeviceConfig(std::string device_name, std::shared_ptr<const DeviceConfig> config_ptr) const {
		if(device_configs_.count(device_name) == 1){
			config_ptr = device_configs_.at(device_name);
			return true;
		}
	}

	std::unordered_map<std::string, std::shared_ptr<const DeviceConfig> >::const_iterator begin() const {
		return device_configs_.begin();
	}

	std::unordered_map<std::string, std::shared_ptr<const DeviceConfig> >::const_iterator end() const {
		return device_configs_.end();
	}

	size_t size() const {
		return device_configs_.size();
	}

	bool contains(std::string device_name) const {
		return device_configs_.count(device_name) != 0;
	}

	bool operator==(const DeviceConfigMap& rhs) const {
		bool result = (size() == rhs.size());
		for(const auto& [key, val] : device_configs_){
			if(rhs.contains(key)){
				result &= ((*val) == *(rhs.getDeviceConfig(key)));
			}
			else{
				return false;
			}
		}
		return result;
	}

private:
	std::unordered_map < std::string, std::shared_ptr<const DeviceConfig> > device_configs_;
	std::unordered_map<int, std::string> port_to_device_name_map_;
};

} // namespace ghost_v5_interfaces