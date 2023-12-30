#pragma once

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <ghost_msgs/msg/v5_actuator_command.hpp>
#include <ghost_msgs/msg/v5_sensor_update.hpp>
#include <ghost_serial/base_interfaces/jetson_serial_base.hpp>

#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include <ghost_v5_interfaces/robot_hardware_interface.hpp>

namespace ghost_ros_interfaces {

class JetsonV5SerialNode : public rclcpp::Node {
public:
	JetsonV5SerialNode();
	~JetsonV5SerialNode();

	bool initSerial();

private:
	// Process incoming/outgoing msgs w/ ROS
	void actuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg);
	void publishV5SensorUpdate(const std::vector<unsigned char>& buffer);

	// Background thread for processing serial data and maintaining serial connection
	void serialLoop();

	// Background thread to periodically check if serial data has timed out
	void serialTimeoutLoop();

	// ROS Parameters
	bool use_checksum_;
	bool verbose_;
	std::string read_msg_start_seq_;
	std::string write_msg_start_seq_;
	std::string port_name_;
	std::string backup_port_name_;

	// ROS Topics
	rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;
	rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_pub_;

	// Serial Interface
	std::shared_ptr<ghost_serial::JetsonSerialBase> serial_base_interface_;
	std::vector<unsigned char> sensor_update_msg_;
	std::thread serial_thread_;
	std::thread serial_timeout_thread_;
	std::atomic_bool serial_open_;
	std::chrono::time_point<std::chrono::system_clock> last_msg_time_;
	std::mutex serial_reset_mutex_;
	bool using_backup_port_;

	// Robot Hardware Interface
	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;

	// Msg Config
	int actuator_command_msg_len_;
	int sensor_update_msg_len_;
};

} // namespace ghost_ros_interfaces