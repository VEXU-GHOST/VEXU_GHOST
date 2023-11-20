#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace ghost_v5_core {

// List available V5 Devices
enum device_type_e {
	MOTOR,
	ROTATION_SENSOR,
	INERTIAL_SENSOR, // Unsupported
	DISTANCE_SENSOR, // Unsupported
	OPTICAL_SENSOR,  // Unsupported
	VISION_SENSOR,   // Unsupported
	GPS_SENSOR,      // Unsupported
	RADIO           // Unsupported
};

const std::unordered_map<std::string, device_type_e> device_type_name_enum_map{
	{"MOTOR",           device_type_e::MOTOR},
	{"ROTATION_SENSOR", device_type_e::ROTATION_SENSOR},
	{"INERTIAL_SENSOR", device_type_e::INERTIAL_SENSOR},
	{"DISTANCE_SENSOR", device_type_e::DISTANCE_SENSOR},
	{"OPTICAL_SENSOR",  device_type_e::OPTICAL_SENSOR},
	{"VISION_SENSOR",   device_type_e::VISION_SENSOR},
	{"GPS_SENSOR",      device_type_e::GPS_SENSOR},
	{"RADIO",           device_type_e::RADIO},
};

class DeviceConfig {
public:
	virtual bool operator==(const DeviceConfig&) const {
		return true;
	}
};

class DeviceData {
public:
	virtual bool operator==(const DeviceData&) const {
		return true;
	}
};

class DeviceInterface {
public:

	int port;
	std::string name;
	device_type_e type;
	std::shared_ptr<DeviceConfig> config;
	std::shared_ptr<DeviceData> data;

	bool operator==(const DeviceInterface& rhs) const {
		return (port == rhs.port) && (name == rhs.name) && (type == rhs.type) && (*config == *rhs.config) &&
		       (*data == *rhs.data);
	}
};

using DeviceInterfaceMap = std::unordered_map < std::string, std::shared_ptr<DeviceInterface> >;

} // namespace ghost_v5_core