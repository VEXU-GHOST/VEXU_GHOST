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

class DeviceConfig {
};

class DeviceData {
};

class DeviceInterface {
public:
	int port;
	device_type_e type;
	std::shared_ptr<DeviceConfig> config;
	std::shared_ptr<DeviceData> data;
};

using DeviceConfigMap = std::unordered_map < std::string, std::shared_ptr<DeviceInterface> >;

} // namespace ghost_v5_core