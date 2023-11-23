#pragma once

#include "../../filters/second_order_low_pass_filter.hpp"
#include "../../motor/dc_motor_model.hpp"
#include "../../motor/motor_controller.hpp"
#include "../base/device_interfaces.hpp"

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

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<MotorDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const MotorDeviceConfig *m_rhs = dynamic_cast<const MotorDeviceConfig *>(&rhs);
		if(m_rhs){
			return (port == m_rhs->port) && (name == m_rhs->name) && (type == m_rhs->type) && (reversed == m_rhs->reversed) &&
			       (encoder_units == m_rhs->encoder_units) && (gearset == m_rhs->gearset) &&
			       (brake_mode == m_rhs->brake_mode) && (controller_config == m_rhs->controller_config) &&
			       (filter_config == m_rhs->filter_config) && (model_config == m_rhs->model_config);
		}
		else{
			// Failed to cast base class, thus can't be equal.
			return false;
		}
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