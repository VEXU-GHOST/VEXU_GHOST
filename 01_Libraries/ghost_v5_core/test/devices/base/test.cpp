#include <memory>
#include "ghost_v5_core/devices/base/device_config_map.hpp"
#include "ghost_v5_core/devices/motor/motor_device_config.hpp"

using ghost_v5_core::DeviceConfigMap;
using ghost_v5_core::MotorDeviceConfig;

extern "C" DeviceConfigMap* factory(void) {
	DeviceConfigMap* robot_config = new DeviceConfigMap;

	std::shared_ptr<MotorDeviceConfig> motor_1 = std::make_shared<MotorDeviceConfig>();
	motor_1->name = "motor_1";
	robot_config->addDeviceConfig(motor_1);

	return robot_config;
}