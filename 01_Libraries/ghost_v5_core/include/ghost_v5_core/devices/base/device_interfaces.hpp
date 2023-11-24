#pragma once

#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

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

class DeviceBase : public std::enable_shared_from_this<DeviceBase> {
public:
	std::string name = "";
	device_type_e type = device_type_e::INVALID;

	/**
	 * @brief Allows for deep-copying of DeviceBase.
	 *
	 * @return std::shared_ptr<DeviceBase> pointer to a copy of the original object
	 */
	virtual std::shared_ptr<DeviceBase> clone() const = 0;

	/**
	 * @brief Helper to convert base class pointer to derived class pointer.
	 * DeviceBase is pure virtual, so a DeviceBase pointer is always actually pointing to a Derived class.
	 *
	 * @return std::shared_ptr<Derived> pointer to this object as its derived class
	 */
	template<typename Derived>
	std::shared_ptr<Derived> as() {
		auto derived_ptr = std::dynamic_pointer_cast<Derived>(shared_from_this());
		if(derived_ptr == nullptr){
			throw std::runtime_error("[DeviceBase::as] Error: Cannot downcast device " + name + "!");
		}
		return derived_ptr;
	}

	/**
	 * @brief Helper to convert const base class pointer to const derived class pointer.
	 * DeviceBase is pure virtual, so a DeviceBase pointer is always actually pointing to a Derived class.
	 *
	 * @return std::shared_ptr<const Derived> pointer to this object as its derived class
	 */
	template<typename Derived>
	std::shared_ptr<const Derived> as() const {
		auto derived_ptr = std::dynamic_pointer_cast<const Derived>(shared_from_this());
		if(derived_ptr == nullptr){
			throw std::runtime_error("[DeviceBase::as] Error: Cannot downcast device " + name + "!");
		}
		return derived_ptr;
	}

	virtual bool operator==(const DeviceBase &rhs) const = 0;
};

class DeviceConfig : public DeviceBase {
public:
	int port = 0;
	virtual std::shared_ptr<DeviceBase> clone() const = 0;
	virtual bool operator==(const DeviceBase &rhs) const = 0;
};

class DeviceData : public DeviceBase {
public:
	virtual std::shared_ptr<DeviceBase> clone() const = 0;
	virtual bool operator==(const DeviceBase &rhs) const = 0;
	virtual std::vector<unsigned char> serialize(bool to_v5) const = 0;
	virtual void deserialize(const std::vector<unsigned char>& data, bool from_coprocessor)  = 0;
};

} // namespace ghost_v5_core