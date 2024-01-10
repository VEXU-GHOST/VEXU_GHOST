#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "ghost_autonomy/run_tree.hpp"
#include "std_msgs/msg/bool.hpp"

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

	node_ptr_->declare_parameter("bt_topic", "/behavior_tree/auton");
	std::string bt_topic = node_ptr_->get_parameter("bt_topic").as_string();

	robot_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		pose_topic,
		10,
		std::bind(&SwerveRobotPlugin::poseUpdateCallback, this, _1)
		);

	odom_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
		odom_topic,
		10);

	bt_auton_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
		bt_topic,
		10);
}
void SwerveRobotPlugin::disabled(){
}
void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;
	// std::shared_ptr<RunTreeNode> tree_node = std::make_shared<RunTreeNode>(robot_hardware_interface_ptr_);
	// auto node = std::make_shared<RunTreeNode>();
	// rclcpp::spin(tree_node);
	// rclcpp::shutdown();

	// make publisher to say start auton
	std_msgs::msg::Bool msg{};
	msg.data = true;
	bt_auton_pub_->publish(msg);
}
void SwerveRobotPlugin::teleop(double current_time){
	std::cout << "Teleop: " << current_time << std::endl;
	auto joy_data = robot_hardware_interface_ptr_->getMainJoystickData();

	std::cout << joy_data->right_y << std::endl;

	robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_frr", 2500);
	robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_frr", joy_data->right_y / 127.0);
}

void SwerveRobotPlugin::poseUpdateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)