#include <iostream>
#include <ghost_swerve/swerve_model.hpp>
#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

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

	m_robot_pose_sub = node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		pose_topic,
		10,
		std::bind(&SwerveRobotPlugin::poseUpdateCallback, this, _1)
		);

	m_odom_pub = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
		odom_topic,
		10);

	// Setup Swerve Model
	SwerveConfig swerve_model_config;
	swerve_model_config.max_wheel_lin_vel = 2.75 * M_PI * 650 / 60;
	swerve_model_config.module_type = swerve_type_e::DIFFERENTIAL;
	swerve_model_config.steering_ratio = 13.0 / 44.0;
	swerve_model_config.wheel_ratio = swerve_model_config.steering_ratio * 30.0 / 14.0;

	swerve_model_config.module_positions["left_front"] = Eigen::Vector2d(0.1143, 0.1143);
	swerve_model_config.module_positions["right_front"] = Eigen::Vector2d(0.1143, -0.1143);
	swerve_model_config.module_positions["left_back"] = Eigen::Vector2d(-0.1143, 0.1143);
	swerve_model_config.module_positions["right_back"] = Eigen::Vector2d(-0.1143, -0.1143);

	m_swerve_model = std::make_shared<SwerveModel>(swerve_model_config);
}

void SwerveRobotPlugin::onNewSensorData(){
	// updateRobotStates()
}

void SwerveRobotPlugin::disabled(){
}

void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;
}
void SwerveRobotPlugin::teleop(double current_time){
	std::cout << "Teleop: " << current_time << std::endl;
	auto joy_data = robot_hardware_interface_ptr_->getMainJoystickData();
}

void SwerveRobotPlugin::poseUpdateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)