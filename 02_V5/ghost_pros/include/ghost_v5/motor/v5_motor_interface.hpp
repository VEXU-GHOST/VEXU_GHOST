#pragma once
#include "ghost_control/motor_controller.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "pros/apix.h"
#include "pros/error.h"
#include "pros/motors.hpp"

#include <unordered_map>

using ghost_control::MotorController;
using ghost_v5_interfaces::MotorDeviceConfig;

namespace ghost_v5 {

class V5MotorInterface : public MotorController {
public:
	V5MotorInterface(std::shared_ptr<const MotorDeviceConfig> config_ptr);

	void updateInterface();

	bool getDeviceIsConnected(){
		return device_connected_;
	}

	void setCurrentLimit(int32_t current_limit_ma){
		motor_interface_ptr_->set_current_limit(current_limit_ma);
	}

	std::shared_ptr<pros::Motor> getMotorInterfacePtr(){
		return motor_interface_ptr_;
	}

private:
	std::shared_ptr<pros::Motor> motor_interface_ptr_;
	bool device_connected_;
	std::shared_ptr<const MotorDeviceConfig> config_ptr_;
};

} // namespace ghost_v5