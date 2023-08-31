#include "ghost_competition_ros/v5_robot_base.hpp"

using std::placeholders::_1;

namespace ghost_competition_ros {

void V5RobotBase::configure(const std::string& robot_name, const YAML::Node& config_yaml){
	config_yaml_ = config_yaml;
	node_ptr_ = std::make_shared<rclcpp::Node>(robot_name + "_node");

	sensor_update_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::V5SensorUpdate>(
		"/v5/sensor_update",
		10,
		std::bind(&V5RobotBase::sensorUpdateCallback, this, _1)
		);

	actuator_command_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::V5ActuatorCommand>(
		"/v5/actuator_command",
		10);

	start_time_ = std::chrono::system_clock::now();

	initialize();
	configured_ = true;
}

void V5RobotBase::sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
	updateCompetitionState(msg->is_disabled, msg->is_autonomous);

	// Competition State Machine
	switch(curr_comp_state_){
	case robot_state_e::DISABLED:
		disabled();
		break;

	case robot_state_e::AUTONOMOUS:
		autonomous(getTimeFromStart());
		break;

	case robot_state_e::TELEOP:
		teleop(getTimeFromStart());
		break;
	}
}

void V5RobotBase::updateCompetitionState(bool is_disabled, bool is_autonomous){
	// Update state with new value
	if(is_disabled){
		curr_comp_state_ = robot_state_e::DISABLED;
	}
	else if(is_autonomous){
		curr_comp_state_ = robot_state_e::AUTONOMOUS;
	}
	else{
		curr_comp_state_ = robot_state_e::TELEOP;
	}

	// Process state transitions
	if((curr_comp_state_ == robot_state_e::AUTONOMOUS) && (last_comp_state_ == robot_state_e::DISABLED)){
		// DISABLED -> AUTONOMOUS
		start_time_ = std::chrono::system_clock::now();
	}
	if((curr_comp_state_ == robot_state_e::TELEOP) && (last_comp_state_ != robot_state_e::TELEOP)){
		// DISABLED/AUTONOMOUS -> TELEOP
		start_time_ = std::chrono::system_clock::now();
	}

	last_comp_state_ = curr_comp_state_;
}

double V5RobotBase::getTimeFromStart() const {
	auto curr_time = std::chrono::system_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - start_time_).count() / 1000.0;
}

} // namespace ghost_competition_ros