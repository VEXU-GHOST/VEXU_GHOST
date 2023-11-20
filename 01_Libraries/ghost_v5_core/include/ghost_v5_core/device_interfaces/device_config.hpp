#pragma once

#include <memory>
#include <string>
#include <unordered_map>

namespace ghost_v5_core {

// List available V5 Devices
enum device_type_e {
	Motor,
	RotationSensor,
	InertialSensor, // Unsupported
	DistanceSensor, // Unsupported
	OpticalSensor,  // Unsupported
	VisionSensor,   // Unsupported
	GPSSensor,      // Unsupported
	Radio           // Unsupported
};

class DeviceConfigBase {
public:
	int port;
	std::string device_name;
	device_type_e device_type;
};

using DeviceConfigMap = std::unordered_map < std::string, std::shared_ptr<DeviceConfigBase> >;

} // namespace ghost_v5_core