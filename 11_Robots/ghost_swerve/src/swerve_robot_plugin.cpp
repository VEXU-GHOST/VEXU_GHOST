#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <iostream>

using std::placeholders::_1;

namespace ghost_swerve {

SwerveRobotPlugin::SwerveRobotPlugin(){
}

void SwerveRobotPlugin::initialize(){
	std::cout << "Swerve Robot Initialization!" << std::endl;

	node_ptr_->declare_parameter("pose_topic", "/estimation/pose");
	std::string pose_topic = node_ptr_->get_parameter("pose_topic").as_string();

	node_ptr_->declare_parameter("odom_topic", "/sensors/odom");
	std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

	robot_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		pose_topic,
		10,
		std::bind(&SwerveRobotPlugin::poseUpdateCallback, this, _1)
		);

	odom_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
		odom_topic,
		10);
}
void SwerveRobotPlugin::disabled(){
}
void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;
}
void SwerveRobotPlugin::teleop(double current_time){
	std::cout << "Teleop: " << current_time << std::endl;
	auto joy_data = robot_hardware_interface_ptr_->getMainJoystickData();

	if(joy_data->btn_a){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_flr", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_flr", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_b){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_fll", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_fll", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_x){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_frr", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_frr", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_y){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_frl", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_frl", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_u){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_blf", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_blf", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_l){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_blb", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_blb", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_r){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_brb", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_brb", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_d){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_brf", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_brf", joy_data->right_y / 127.0);
	}
}

void SwerveRobotPlugin::poseUpdateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)