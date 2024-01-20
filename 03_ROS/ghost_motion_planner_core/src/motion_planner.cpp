#include "ghost_motion_planner_core/motion_planner.hpp"

using std::placeholders::_1;

namespace ghost_motion_planner {

void MotionPlanner::configure(){
	// std::cout << "Configuring Motion Planner" << std::endl;
	node_ptr_ = std::make_shared<rclcpp::Node>("motion_planner_node");

	// loadRobotHardwareInterface();
    // probably need this for motor names?

	sensor_update_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::V5SensorUpdate>(
		"/v5/sensor_update",
		10,
		std::bind(&MotionPlanner::sensorUpdateCallback, this, _1)
		);

	trajectory_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::RobotTrajectory>(
		"motionPlanning/trajectory",
		10);

	// start_time_ = std::chrono::system_clock::now();

	initialize();
	// configured_ = true;
}

void MotionPlanner::sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
	// updateCompetitionState(msg->competition_status.is_disabled, msg->competition_status.is_autonomous);
	// fromROSMsg(*robot_hardware_interface_ptr_, *msg);
    
    // update values for trajectory calculation
}

void MotionPlanner::setNewCommand(ghost_msgs::msg::DrivetrainCommand command){
    // generateMotionPlan(command);
    // does anything else go here?
    // choose teleop vs auton version?
}


} // namespace ghost_motion_planner