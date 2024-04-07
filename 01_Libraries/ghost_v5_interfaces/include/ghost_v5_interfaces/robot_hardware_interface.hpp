#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include "ghost_v5_interfaces/devices/device_config_map.hpp"
#include "ghost_v5_interfaces/devices/inertial_sensor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/joystick_device_interface.hpp"
#include "ghost_v5_interfaces/devices/motor_device_interface.hpp"
#include "ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp"


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

	/**
	 * @brief Checks equality without evaluating hardware_type (for testing if data is reconstructed correctly after transport).
	 */
	bool isDataEqual(const RobotHardwareInterface& rhs) const;

	/**
	 * @brief Compares all parameters for equality
	 */
	bool operator==(const RobotHardwareInterface& rhs) const;

	//////////////////////////////////////////////////////////////
	///////////////////// Competition Status /////////////////////
	//////////////////////////////////////////////////////////////

	/**
	 * @brief Returns if robot is currently disabled via field control.
	 */
	bool isDisabled() const {
		return is_disabled_;
	}

	/**
	 * @brief Returns if robot is currently in autonomous mode via field control.
	 */
	bool isAutonomous() const {
		return is_autonomous_;
	}

	/**
	 * @brief Returns if robot is currently connected to field control.
	 */
	bool isConnected() const {
		return is_connected_;
	}

	/**
	 * @brief Set disabled status from V5 Brain.
	 * This is only used to report current status from the V5 Brain to the coprocessor,
	 * it won't change anything on the V5 Brain.
	 *
	 * @param is_disabled
	 */
	void setDisabledStatus(bool is_disabled){
		is_disabled_ = is_disabled;
	}

	/**
	 * @brief Set autonomous status from V5 Brain.
	 * This is only used to report current status from the V5 Brain to the coprocessor,
	 * it won't change anything on the V5 Brain.
	 *
	 * @param is_autonomous
	 */
	void setAutonomousStatus(bool is_autonomous){
		is_autonomous_ = is_autonomous;
	}

	/**
	 * @brief Set connected status from V5 Brain.
	 * This is only used to report current status from the V5 Brain to the coprocessor,
	 * it won't change anything on the V5 Brain.
	 *
	 * @param is_connected
	 */
	void setConnectedStatus(bool is_connected){
		is_connected_ = is_connected;
	}

	//////////////////////////////////////////////////////////////
	////////////////////// Motor Interfaces //////////////////////
	//////////////////////////////////////////////////////////////

	/**
	 * @brief Returns motor position in the configured encoder units
	 *
	 * @param motor_name
	 * @return float
	 */
	float getMotorPosition(const std::string& motor_name);

	/**
	 * @brief Sets motor position in the configured encoder units.
	 *
	 * @param motor_name
	 * @param position_cmd
	 */
	void setMotorPositionCommand(const std::string& motor_name, float position_cmd);

	float getMotorVelocityRPM(const std::string& motor_name);
	void setMotorVelocityCommandRPM(const std::string& motor_name, float velocity_cmd);
	void setMotorVoltageCommandPercent(const std::string& motor_name, float voltage_cmd);
	void setMotorTorqueCommandPercent(const std::string& motor_name, float torque_cmd);

	/**
	 * @brief Updates controller setpoints. Requires SetMotorControlMode to enable different controller terms.
	 *
	 * @param motor_name
	 * @param position_cmd 	Configured encoder units
	 * @param velocity_cmd	RPM
	 * @param voltage_cmd 	Percent (-1.0 -> 1.0)
	 * @param torque_cmd 	Percent (-1.0 -> 1.0)
	 */
	void setMotorCommand(const std::string& motor_name,
	                     float position_cmd,
	                     float velocity_cmd,
	                     float voltage_cmd,
	                     float torque_cmd = 0.0);

	/**
	 * @brief Updates which control terms are active. Used along with setMotorCommand.
	 *
	 * @param motor_name
	 * @param position_control
	 * @param velocity_control
	 * @param voltage_control
	 * @param torque_control
	 */
	void setMotorControlMode(const std::string& motor_name,
	                         bool position_control,
	                         bool velocity_control,
	                         bool voltage_control,
	                         bool torque_control);

	/**
	 * @brief Set the maximum motor current limit (clamped at 2500mA).
	 * Current limits will default to zero to optimize power allocation. This needs to be set in order to use a motor!
	 *
	 * @param motor_name
	 * @param current_limit_ma
	 */
	void setMotorCurrentLimitMilliAmps(const std::string& motor_name, int32_t current_limit_ma);

	//////////////////////////////////////////////////////////////
	///////////////// Rotation Sensor Interfaces /////////////////
	//////////////////////////////////////////////////////////////

	float getRotationSensorAngleDegrees(const std::string& sensor_name);
	float getRotationSensorPositionDegrees(const std::string& sensor_name);
	float getRotationSensorVelocityRPM(const std::string& sensor_name);

	//////////////////////////////////////////////////////////////
	///////////////// Inertial Sensor Interfaces /////////////////
	//////////////////////////////////////////////////////////////
	float getInertialSensorXRate(const std::string& sensor_name);
	float getInertialSensorYRate(const std::string& sensor_name);
	float getInertialSensorZRate(const std::string& sensor_name);
	float getInertialSensorXAccel(const std::string& sensor_name);
	float getInertialSensorYAccel(const std::string& sensor_name);
	float getInertialSensorZAccel(const std::string& sensor_name);
	float getInertialSensorHeading(const std::string& sensor_name);

	//////////////////////////////////////////////////////////////
	///////////////// Joystick Device Interfaces /////////////////
	//////////////////////////////////////////////////////////////
	std::shared_ptr<devices::JoystickDeviceData> getMainJoystickData();
	std::shared_ptr<devices::JoystickDeviceData> getPartnerJoystickData();

	//////////////////////////////////////////////////////////////
	///////////////////////// Digital IO /////////////////////////
	//////////////////////////////////////////////////////////////

	void setDigitalIO(const std::vector<bool>& digital_io);
	const std::vector<bool>& getDigitalIO() const;

	/////////////////////////////////////////////////////////
	/////////////////// Device Interfaces ///////////////////
	/////////////////////////////////////////////////////////

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

	bool contains(const std::string& device_name) const {
		return device_pair_name_map_.count(device_name) != 0;
	}

	/**
	 * @brief Returns the device pair (config and data) for a given device.
	 * This is a deep-copy, changing device data should go through the setDeviceData interface.
	 *
	 * Throws a runtime error if the device is not found.
	 *
	 * @param device_name
	 * @return devices::DevicePair
	 */
	devices::DevicePair getDevicePair(const std::string& device_name) const;

	/**
	 * @brief Updates a given Device with new Data using the device name.
	 *
	 * Throws a runtime error if the device is not found.
	 *
	 * @param device_data
	 */
	void setDeviceData(std::shared_ptr<devices::DeviceData> device_data);

	/**
	 * @brief Returns a pointer to a Device's configuration given the device name.
	 * Device Configurations are declared const and thus are read-only.
	 *
	 * Throws a runtime error if the device is not found.
	 *
	 * Throws a runtime error if the device cannot be cast to the template type.
	 * 		Example: getDeviceConfig<MotorDeviceConfig>("my_rotation_sensor")
	 *
	 * @tparam T derived class type
	 * @param device_name
	 * @return std::shared_ptr<const T>
	 */
	template <typename T>
	std::shared_ptr<const T> getDeviceConfig(const std::string& device_name) const {
		static_assert(std::is_base_of<devices::DeviceConfig, T>::value, "Template parameter is not derived from DeviceConfig! Did you mean getDeviceData?");
		throwOnNonexistentDevice(device_name);
		return device_pair_name_map_.at(device_name).config_ptr->as<T>();
	}

	/**
	 * @brief Returns a pointer to a copy of a Device's data given the device name.
	 * This is a deep-copy, changing device data should go through the setDeviceData interface.
	 *
	 * Throws a runtime error if the device is not found.
	 *
	 * Throws a runtime error if the device cannot be cast to the template type.
	 * 		Example: getDeviceData<MotorDeviceData>("my_rotation_sensor")
	 *
	 * @tparam T derived class type
	 * @param device_name
	 * @return std::shared_ptr<const T>
	 */
	template <typename T>
	std::shared_ptr<T> getDeviceData(const std::string& device_name) const {
		static_assert(std::is_base_of<devices::DeviceData, T>::value, "Template parameter is not derived from DeviceData! Did you mean getDeviceConfig?");
		throwOnNonexistentDevice(device_name);
		return device_pair_name_map_.at(device_name).data_ptr->clone()->as<T>();
	}

	/////////////////////////////////////////////////////////////
	/////////////////////// Serialization ///////////////////////
	/////////////////////////////////////////////////////////////

	/**
	 * @brief returns Msg ID of most recent update
	 *
	 * @return int
	 */
	int getMsgID() const {
		return msg_id_;
	}

	/**
	 * @brief Set Msg ID of most recent update
	 *
	 * @param msg_id
	 */
	void setMsgID(int msg_id){
		msg_id_ = msg_id;
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
	 * Throws a runtime error if the msg buffer does not match the expected msg length.
	 *
	 * @return std::vector<unsigned char>
	 */
	std::vector<unsigned char> serialize() const;

	/**
	 * @brief Updates all device date from a single byte stream.
	 *
	 * Throws a runtime error if the msg buffer does not match the expected msg length.
	 *
	 * Throws a runtime error if the hardware_type is unsupported.
	 *
	 * @param msg
	 * @return int number of bytes processed
	 */
	int deserialize(const std::vector<unsigned char>& msg);

private:
	void setDeviceDataNoLock(std::shared_ptr<devices::DeviceData> device_data);
	void throwOnNonexistentDevice(const std::string& device_name) const;
	devices::hardware_type_e hardware_type_;

	// Competition Status
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

	// Device Data
	std::unordered_map<std::string, devices::DevicePair> device_pair_name_map_;
	std::map<int, devices::DevicePair> device_pair_port_map_;
	std::vector<std::string> device_names_ordered_by_port_;
	std::map<int, std::string> port_to_device_name_map_;
	std::shared_ptr<devices::DeviceConfigMap> robot_config_ptr_;
};

} // namespace ghost_v5_interfaces