#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_msgs/msg/drivetrain_command.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

#include <ghost_v5_interfaces/robot_hardware_interface.hpp>

namespace ghost_motion_planner {

// enum class robot_state_e {
// 	DISABLED = 0,
// 	TELEOP = 1,
// 	AUTONOMOUS = 2,
// };

class MotionPlanner {
public:
	MotionPlanner() = default;
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
	virtual bool generateMotionPlan() = 0; 



	//////////////////////////////
	///// Base Class Methods /////
	//////////////////////////////

	/**
	 * @brief Called for all motion planner classes after construction. Calls user-defined intialize method internally.
	 */
	void configure();
	
	void setNewCommand(ghost_msgs::msg::DrivetrainCommand command);

	// returns trajectory after generateMotionPlan has completed
	// should just publish instead?
	// void getRobotTrajectory();

protected:
	std::shared_ptr<rclcpp::Node> node_ptr_;
	std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;

private:
	// void loadRobotHardwareInterface();
	// void sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
	void sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);

	// bool configured_ = false;
	// robot_state_e last_comp_state_ = robot_state_e::TELEOP;
	// robot_state_e curr_comp_state_ = robot_state_e::TELEOP;
	rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_sub_;
	rclcpp::Publisher<ghost_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;

	// std::chrono::time_point<std::chrono::system_clock> start_time_;
};

} // namespace ghost_motion_planner