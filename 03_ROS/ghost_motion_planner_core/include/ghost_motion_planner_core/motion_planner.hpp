#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_msgs/msg/drivetrain_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

#include <ghost_v5_interfaces/robot_hardware_interface.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

namespace ghost_motion_planner {

class MotionPlanner : public rclcpp::Node {
public:
	MotionPlanner():
	rclcpp::Node("motion_planner"){
		configure();
	};
	virtual ~MotionPlanner() = default;

	///////////////////////////
	///// Virtual Methods /////
	///////////////////////////
	/**
	 * @brief Called directly after instantiation, when the robot is configured.
	 *
	 * Member variables provide access to ROS Node for adding ROS Interfaces (topics, services, etc.) and for loading
	 * ROS Params at runtime.
	 *
	 * This will block any other competition functionality until it completes.
	 * This method will not update actuator commands.
	 */
	virtual void initialize() = 0;
	
	// Blocking, ignore sensor updates while making new plan
	// overload for teleop and auton?
	/**
	 * @brief Called when a DrivetrainCommand msg is recieved 
	 * 
	 * Generates and publishes a RobotTrajectory msg when completed
	 * 
	 */
    virtual void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) = 0;


	//////////////////////////////
	///// Base Class Methods /////
	//////////////////////////////

	/**
	 * @brief Called for all motion planner classes after construction. Calls user-defined intialize method internally.
	 */
	void configure();

	void setNewCommand(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd);

protected:
	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
	rclcpp::Publisher<ghost_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;

private:
	void sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
	void loadRobotHardwareInterface();

	// bool configured_ = false;
	std::atomic_bool planning_ = false;
	rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_sub_;
	rclcpp::Subscription<ghost_msgs::msg::DrivetrainCommand>::SharedPtr pose_command_sub_;

	// std::chrono::time_point<std::chrono::system_clock> start_time_;
};

} // namespace ghost_motion_planner