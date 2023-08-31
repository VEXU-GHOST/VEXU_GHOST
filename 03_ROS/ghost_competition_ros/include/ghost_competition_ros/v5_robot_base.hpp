#pragma once

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

namespace ghost_competition_ros {

enum class robot_state_e {
	DISABLED = 0,
	TELEOP = 1,
	AUTONOMOUS = 2,
};

class V5RobotBase {
public:
	V5RobotBase() = default;
	virtual ~V5RobotBase() = default;

	///////////////////////////
	///// Virtual Methods /////
	///////////////////////////
	/**
	 * @brief Called directly after instantiation, when the robot is configured.
	 *
	 * Member variables for the ROS node and YAML config allow for adding new ros interfaces
	 * and for loading parameters at runtime.
	 *
	 * This will block any other competition functionality until it completes.
	 * This method will not update actuator commands.
	 */
	virtual void initialize() = 0;

	/**
	 * @brief Called while robot is disabled.
	 *
	 * This method will not update actuator commands.
	 */
	virtual void disabled() = 0;

	/**
	 * @brief Performs robot autonomous functionality when new Sensor Data is received.
	 * Function is called at roughly 100Hz and should not block.
	 *
	 * @param current_time time in seconds since the start of the autonomous period.
	 */
	virtual void autonomous(double current_time) = 0;

	/**
	 * @brief Performs robot teleoperation when new Sensor Data is received.
	 * Function is called at roughly 100Hz and should not block.
	 *
	 * @param current_time time in seconds since the start of the teleop period.
	 */
	virtual void teleop(double current_time) = 0;

	//////////////////////////////
	///// Base Class Methods /////
	//////////////////////////////
	/**
	 * @brief Called for all V5 Robot Classes after construction. Calls user-defined intialize method internally.
	 *
	 * @param node_ptr      pointer to ros node
	 * @param config_yaml   YAML configuration node
	 */
	void configure(const std::string& robot_name, const YAML::Node& config_yaml);

	/**
	 * @brief Returns a shared pointer to the ROS node for this robot instance
	 *
	 * @return std::shared_ptr<rclcpp::Node>
	 */
	std::shared_ptr<rclcpp::Node> getROSNodePtr() const {
		return node_ptr_;
	}

	/**
	 * @brief Returns a constant reference to the configuration yaml node
	 *
	 * @return const YAML::Node&
	 */
	const YAML::Node& getConfigYAML() const {
		return config_yaml_;
	}

private:
	void sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
	void updateCompetitionState(bool is_disabled, bool is_autonomous);
	double getTimeFromStart() const;

	std::shared_ptr<rclcpp::Node> node_ptr_;
	YAML::Node config_yaml_;
	bool configured_ = false;
	robot_state_e last_comp_state_ = robot_state_e::TELEOP;
	robot_state_e curr_comp_state_ = robot_state_e::TELEOP;
	rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_sub_;
	rclcpp::Publisher<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_pub_;

	std::chrono::time_point<std::chrono::system_clock> start_time_;
};

} // namespace ghost_competition_ros