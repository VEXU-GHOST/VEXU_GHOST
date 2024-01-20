#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iostream>

using std::placeholders::_1;

namespace ghost_swerve {

SwerveRobotPlugin::SwerveRobotPlugin(){
}

void SwerveRobotPlugin::initialize(){
	std::cout << "Swerve Robot Initialization!" << std::endl;

	node_ptr_->declare_parameter("trajectory_topic", "/motion_planning/trajectory");
	std::string trajectory_topic = node_ptr_->get_parameter("trajectory_topic").as_string();

	node_ptr_->declare_parameter("odom_topic", "/sensors/odom");
	std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

	// node_ptr_->declare_parameter("bt_topic", "/behavior_tree/auton");
	// std::string bt_topic = node_ptr_->get_parameter("bt_topic").as_string();

	node_ptr_->declare_parameter<std::string>("bt_path");
	bt_path_ = node_ptr_->get_parameter("bt_path").as_string();

	trajectory_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::RobotTrajectory>(
		trajectory_topic,
		10,
		std::bind(&SwerveRobotPlugin::trajectoryCallback, this, _1)
		);

	odom_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
		odom_topic,
		10);

	bt_ = std::make_shared<RunTree>(bt_path_, robot_hardware_interface_ptr_);
}
void SwerveRobotPlugin::disabled(){
}
void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;

	bt_->tick_tree();
}
void SwerveRobotPlugin::teleop(double current_time){
	// std::cout << "Teleop: " << current_time << std::endl;

	// for testing
	autonomous(current_time);

	// auto joy_data = robot_hardware_interface_ptr_->getMainJoystickData();

	// std::cout << joy_data->right_y << std::endl;

	// robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_frr", 2500);
	// robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_frr", joy_data->right_y / 127.0);
}

// does this need to be a part of the base class?
void SwerveRobotPlugin::trajectoryCallback(const ghost_msgs::msg::RobotTrajectory::SharedPtr msg){
	// call interpolator?
	// set trajectory to follow
	// auton/teleop can use trajectory as a time function
}

// void interpolator()


} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)