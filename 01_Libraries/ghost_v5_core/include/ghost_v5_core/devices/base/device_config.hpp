#pragma once

#include <memory>
#include <stdexcept>
#include <ghost_v5_core/devices/base/device_types.hpp>

namespace ghost_v5_core {

class DeviceConfig {
public:

	int port = 0;
	std::string name = "";
	device_type_e type = device_type_e::INVALID;

	/**
	 * @brief Allows for deep-copying of DeviceConfig.
	 *
	 * @return std::shared_ptr<DeviceConfig> pointer to a copy of the original object
	 */
	virtual std::shared_ptr<DeviceConfig> clone() const = 0;

	/**
	 * @brief Helper to convert base class pointer to derived class pointer.
	 * DeviceConfig is pure virtual, so a DeviceConfig pointer is always actually pointing to a Derived class.
	 *
	 * @return std::shared_ptr<Derived> pointer to this object as its derived class
	 */
	template<typename Derived>
	std::shared_ptr<Derived> as(){
		auto derived_ptr = dynamic_cast<Derived*>(this);
		if(derived_ptr == nullptr){
			throw std::runtime_error("[DeviceConfig::as] Error: Cannot downcast device " + name + "!");
		}
		return std::make_shared<Derived>(*derived_ptr);
	}
};

} // namespace ghost_v5_core