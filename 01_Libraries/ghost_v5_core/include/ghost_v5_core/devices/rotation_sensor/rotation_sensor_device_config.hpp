#pragma once

#include "../base/device_interfaces.hpp"

namespace ghost_v5_core {

class RotationSensorDeviceConfig : public DeviceConfig {
public:

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<RotationSensorDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const RotationSensorDeviceConfig *m_rhs = dynamic_cast<const RotationSensorDeviceConfig *>(&rhs);
		if(m_rhs){
			return (port == m_rhs->port) && (name == m_rhs->name) && (type == m_rhs->type) &&
			       (reversed == m_rhs->reversed) && (data_rate == m_rhs->data_rate);
		}
		else{
			// Failed to cast base class, thus can't be equal.
			return false;
		}
	}

	bool reversed = false;
	uint32_t data_rate = 5;
};

} // namespace ghost_v5_core