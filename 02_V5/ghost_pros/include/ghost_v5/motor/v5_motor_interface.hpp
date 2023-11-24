#pragma once
#include "ghost_v5_core/devices/motor/motor_device_interface.hpp"
#include "ghost_v5_core/motor/motor_controller.hpp"
#include "pros/apix.h"
#include "pros/error.h"
#include "pros/motors.hpp"

#include <unordered_map>

using ghost_v5_core::MotorController;
using ghost_v5_core::MotorDeviceConfig;

namespace ghost_v5 {

class V5MotorInterface : public MotorController {
public:
	V5MotorInterface(const MotorDeviceConfig &config);

	void updateInterface();

	bool getDeviceIsConnected(){
		return device_connected_;
	}

	std::shared_ptr<pros::Motor> getMotorInterfacePtr(){
		return motor_interface_ptr_;
	}

private:
	std::shared_ptr<pros::Motor> motor_interface_ptr_;
	bool device_connected_;
};

} // namespace ghost_v5