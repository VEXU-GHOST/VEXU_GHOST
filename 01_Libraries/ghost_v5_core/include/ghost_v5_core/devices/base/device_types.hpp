#pragma once

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
	RADIO,           // Unsupported
	INVALID
};

} // namespace ghost_v5_core