#pragma once

#include "ghost_v5_core/devices/base/device_config.hpp"
#include "ghost_v5_core/filters/second_order_low_pass_filter.hpp"
#include "ghost_v5_core/motor/dc_motor_model.hpp"
#include "ghost_v5_core/motor/motor_controller.hpp"

namespace ghost_v5_core {

enum ghost_gearset {
	GEARSET_100,
	GEARSET_200,
	GEARSET_600
};

enum ghost_brake_mode {
	BRAKE_MODE_COAST,
	BRAKE_MODE_BRAKE,
	BRAKE_MODE_HOLD,
	BRAKE_MODE_INVALID
};

enum ghost_encoder_unit {
	ENCODER_DEGREES,
	ENCODER_ROTATIONS,
	ENCODER_COUNTS,
	ENCODER_INVALID
};

class MotorDeviceConfig : public DeviceConfig {
public:

	std::shared_ptr<DeviceConfig> clone() const override {
		return std::make_shared<MotorDeviceConfig>(*this);
	}

	bool operator==(const MotorDeviceConfig& rhs) const {
		return (encoder_units == rhs.encoder_units) && (gearset == rhs.gearset) && (brake_mode == rhs.brake_mode) &&
		       (controller_config == rhs.controller_config) && (filter_config == rhs.filter_config) &&
		       (model_config == rhs.model_config);
	}

	bool reversed = false;
	MotorController::Config controller_config;
	SecondOrderLowPassFilter::Config filter_config;
	DCMotorModel::Config model_config;

	// These three map 1:1 to their PROS counterpart on the V5 Side.
	ghost_encoder_unit encoder_units{ghost_encoder_unit::ENCODER_COUNTS};
	ghost_gearset gearset{ghost_gearset::GEARSET_200};
	ghost_brake_mode brake_mode{ghost_brake_mode::BRAKE_MODE_COAST};
};

} // namespace ghost_v5_core