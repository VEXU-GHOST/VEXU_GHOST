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

class RobotHardwareInterface {
public:
	RobotHardwareInterface(std::shared_ptr<devices::DeviceConfigMap> robot_config_ptr, devices::hardware_type_e hardware_type);

	bool isDisabled() const {
		return is_disabled_;
	}

	bool isAutonomous() const {
		return is_autonomous_;
	}

	bool isConnected() const {
		return is_connected_;
	}

	int getMsgID() const {
		return msg_id_;
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

	void setMsgID(int msg_id){
		msg_id_ = msg_id;
	}

	bool usesSecondaryJoystick() const {
		return use_secondary_joystick_;
	}

	/**
	 * @brief Returns the device pair (config and data) for a given device.
	 * This is a deep-copy, changing device data should go through the setDeviceDate interface.
	 *
	 * @param name
	 * @return devices::DevicePair
	 */
	devices::DevicePair getDevicePair(const std::string& name) const;

	/**
	 * @brief Returns a pointer to a const Config for a given Device by name.
	 * The config is declared const and thus read-only.
	 *
	 * @param name
	 * @return std::shared_ptr<const devices::DeviceConfig>
	 */
	std::shared_ptr<const devices::DeviceConfig> getDeviceConfig(const std::string& name) const;

	/**
	 * @brief Returns a pointer to the Data for a given Device by name.
	 * This is a deep-copy, changing device data should go through the setDeviceDate interface.
	 *
	 * @param name
	 * @return std::shared_ptr<devices::DeviceData>
	 */
	std::shared_ptr<devices::DeviceData> getDeviceData(const std::string& name) const;

	/**
	 * @brief Returns a pointer to the Data for a given Device by port number.
	 * This is a deep-copy, changing device data should go through the setDeviceDate interface.
	 *
	 * @param port
	 * @return std::shared_ptr<devices::DeviceData>
	 */
	std::shared_ptr<devices::DeviceData> getDeviceData(int port) const;

	/**
	 * @brief Updates a given Device with new Data.
	 * Throws if the device does not exist.
	 *
	 * @param device_data
	 */
	void setDeviceData(std::shared_ptr<devices::DeviceData> device_data);

	/**
	 * @brief Updates a given Device with new Data given device name.
	 * Throws if the device does not exist.
	 *
	 * @param name
	 * @param device_data
	 */
	void setDeviceData(std::string name, std::shared_ptr<devices::DeviceData> device_data);

	/**
	 * @brief Returns a pointer to the Data for the primary joystick.
	 * This is a deep-copy, changing joystick data should go through the setPrimaryJoystickData interface.
	 *
	 * @return std::shared_ptr<devices::JoystickDeviceData>
	 */
	std::shared_ptr<devices::JoystickDeviceData> getPrimaryJoystickData() const;

	/**
	 * @brief Returns a pointer to the data for the secondary joystick.
	 *
	 * @return std::shared_ptr<devices::JoystickDeviceData>
	 */
	std::shared_ptr<devices::JoystickDeviceData> getSecondaryJoystickData() const;

	/**
	 * @brief Update the primary joystick data.
	 *
	 * @param data_ptr
	 */
	void setPrimaryJoystickData(std::shared_ptr<devices::JoystickDeviceData>& data_ptr);

	/**
	 * @brief Update the secondary joystick data.
	 *
	 * @param data_ptr
	 */
	void setSecondaryJoystickData(std::shared_ptr<devices::JoystickDeviceData>& data_ptr);

	/**
	 * @brief Update the digital io ports.
	 *
	 * @param digital_io
	 */
	void setDigitalIO(const std::vector<bool>& digital_io);

	/**
	 * @brief Get values of digital io ports.
	 *
	 * @return const std::vector<bool>&
	 */
	const std::vector<bool>& getDigitalIO() const;

	/**
	 * @brief Returns iterator to first device name (ordered by port number).
	 *
	 * @return std::vector<std::string>::iterator
	 */
	std::vector<std::string>::const_iterator begin() const {
		return device_names_ordered_by_port_.begin();
	}

	/**
	 * @brief Returns iterator to last device name (ordered by port number).
	 *
	 * @return std::vector<std::string>::iterator
	 */
	std::vector<std::string>::const_iterator end() const {
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
	 * @param msg
	 * @return int number of bytes processed
	 */
	int deserialize(const std::vector<unsigned char>& msg);

	bool operator==(const RobotHardwareInterface& rhs) const;

private:

	void throwOnNonexistentDevice(const std::string& name) const;
	devices::hardware_type_e hardware_type_;

	// Competition Statues
	bool is_disabled_ = true;
	bool is_autonomous_ = false;
	bool is_connected_ = false;

	// Digital IO
	std::vector<bool> digital_io_;

	// Serialization
	int msg_id_ = 0;
	int sensor_update_msg_length_;
	int actuator_command_msg_length_;

	// Update Lock
	mutable CROSSPLATFORM_MUTEX_T update_mutex_;

	// Joystick Data
	bool use_secondary_joystick_ = false;
	std::shared_ptr<devices::JoystickDeviceData> primary_joystick_data_ptr_;
	std::shared_ptr<devices::JoystickDeviceData> secondary_joystick_data_ptr_;

	// Device Data
	std::unordered_map<std::string, devices::DevicePair> device_pair_name_map_;
	std::map<int, devices::DevicePair> device_pair_port_map_;
	std::vector<std::string> device_names_ordered_by_port_;
	std::shared_ptr<devices::DeviceConfigMap> robot_config_ptr_;
};

} // namespace ghost_v5_interfaces