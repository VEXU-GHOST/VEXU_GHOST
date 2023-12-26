#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"


#if GHOST_DEVICE == GHOST_JETSON
    #define CROSSPLATFORM_MUTEX_T std::mutex
#elif GHOST_DEVICE == GHOST_V5_BRAIN
    #include "api.h"
    #include "pros/apix.h"
    #define CROSSPLATFORM_MUTEX_T pros::Mutex
#else
    #error "Ghost Device compile flag is not set to valid value"
#endif

namespace ghost_v5_interfaces {

struct DevicePair {
	std::shared_ptr<const DeviceConfig> config_ptr;
	std::shared_ptr<DeviceData> data_ptr;

	DevicePair clone(){
		DevicePair obj;
		obj.config_ptr = config_ptr->clone()->as<const DeviceConfig>();
		obj.data_ptr = data_ptr->clone()->as<DeviceData>();
		return obj;
	}
};

class RobotHardwareInterface {
public:
	RobotHardwareInterface(std::shared_ptr<DeviceConfigMap> robot_config_ptr, hardware_type_e hardware_type);

	bool isDisabled(){
		return is_disabled_;
	}

	bool isAutonomous(){
		return is_autonomous_;
	}

	bool isConnected(){
		return is_connected_;
	}

	void setDisabledStatus(bool is_disabled){
		is_disabled_ = is_disabled;
	}

	void setAutonomousStatus(bool is_autonomous){
		is_autonomous_ = is_autonomous;
	}

	void setConnectedStatus(bool is_connected){
		is_connected_ = is_connected;
	}

	/**
	 * @brief Returns the device pair (config and data) for a given device.
	 * This is a deep-copy, changing device data should go through the setDeviceDate interface.
	 *
	 * @param name
	 * @return DevicePair
	 */
	DevicePair getDevicePair(const std::string& name);

	/**
	 * @brief Returns a pointer to a const Config for a given Device by name.
	 * The config is declared const and thus read-only.
	 *
	 * @param name
	 * @return std::shared_ptr<const DeviceConfig>
	 */
	std::shared_ptr<const DeviceConfig> getDeviceConfig(const std::string& name);

	/**
	 * @brief Returns a pointer to the Data for a given Device by name.
	 * This is a deep-copy, changing device data should go through the setDeviceDate interface.
	 *
	 * @param name
	 * @return std::shared_ptr<DeviceData>
	 */
	std::shared_ptr<DeviceData> getDeviceData(const std::string& name);

	/**
	 * @brief Returns a pointer to the Data for a given Device by port number.
	 * This is a deep-copy, changing device data should go through the setDeviceDate interface.
	 *
	 * @param port
	 * @return std::shared_ptr<DeviceData>
	 */
	std::shared_ptr<DeviceData> getDeviceData(int port);


	/**
	 * @brief Updates a given Device (queried by name) with new Data.
	 * Throws if the device does not exist.
	 *
	 * @param name
	 * @param device_data
	 */
	void setDeviceData(const std::string& name, std::shared_ptr<DeviceData> device_data);

	/**
	 * @brief Returns a pointer to the Data for the primary joystick.
	 * This is a deep-copy, changing joystick data should go through the setPrimaryJoystickData interface.
	 *
	 * @return std::shared_ptr<JoystickDeviceData>
	 */
	std::shared_ptr<JoystickDeviceData> getPrimaryJoystickData();

	/**
	 * @brief Returns a pointer to the data for the secondary joystick.
	 *
	 * @return std::shared_ptr<JoystickDeviceData>
	 */
	std::shared_ptr<JoystickDeviceData> getSecondaryJoystickData();

	/**
	 * @brief Update the primary joystick data.
	 *
	 * @param data_ptr
	 */
	void setPrimaryJoystickData(std::shared_ptr<JoystickDeviceData>& data_ptr);

	/**
	 * @brief Update the secondary joystick data.
	 *
	 * @param data_ptr
	 */
	void setSecondaryJoystickData(std::shared_ptr<JoystickDeviceData>& data_ptr);

	/**
	 * @brief Returns iterator to first device name (ordered by port number).
	 *
	 * @return std::vector<std::string>::iterator
	 */
	std::vector<std::string>::iterator begin(){
		return device_names_ordered_by_port_.begin();
	}

	/**
	 * @brief Returns iterator to last device name (ordered by port number).
	 *
	 * @return std::vector<std::string>::iterator
	 */
	std::vector<std::string>::iterator end(){
		return device_names_ordered_by_port_.end();
	}

	/**
	 * @brief Returns the length of the sensor update byte stream given the current robot configuration.
	 *
	 * @return int
	 */
	int getSensorUpdateMsgLength() const {
		return sensor_update_msg_length_;
	}

	/**
	 * @brief Returns the length of the actuator command byte stream given the current robot configuration.
	 *
	 * @return int
	 */
	int getActuatorCommandMsgLength() const {
		return actuator_command_msg_length_;
	}

	/**
	 * @brief Converts all device data into a single byte stream.
	 *
	 * @return std::vector<unsigned char>
	 */
	std::vector<unsigned char> serialize() const;

	/**
	 * @brief Updates all device date from a single byte stream.
	 *
	 * @return std::vector<unsigned char>
	 */
	void deserialize(std::vector<unsigned char>& msg);

	bool operator==(const RobotHardwareInterface& rhs) const;

private:

	void throwOnNonexistentDevice(const std::string& name);

	hardware_type_e hardware_type_;
	bool is_disabled_ = true;
	bool is_autonomous_ = false;
	bool is_connected_ = false;
	std::vector<bool> digital_io_vector_;
	int sensor_update_msg_length_;
	int actuator_command_msg_length_;
	mutable CROSSPLATFORM_MUTEX_T update_mutex_;

	// Joystick Data
	bool use_secondary_joystick_ = false;
	std::shared_ptr<JoystickDeviceData> primary_joystick_data_ptr_;
	std::shared_ptr<JoystickDeviceData> secondary_joystick_data_ptr_;

	std::unordered_map<std::string, DevicePair> device_pair_name_map_;
	std::map<int, DevicePair> device_pair_port_map_;
	std::vector<std::string> device_names_ordered_by_port_;
	std::shared_ptr<DeviceConfigMap> robot_config_ptr_;
};

} // namespace ghost_v5_interfaces