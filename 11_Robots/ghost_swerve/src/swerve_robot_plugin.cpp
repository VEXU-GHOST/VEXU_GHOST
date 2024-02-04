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

	node_ptr_->declare_parameter("joint_state_topic", "/joint_states");
	std::string joint_state_topic = node_ptr_->get_parameter("joint_state_topic").as_string();

	m_robot_pose_sub = node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		pose_topic,
		10,
		std::bind(&SwerveRobotPlugin::poseUpdateCallback, this, _1)
		);

	m_odom_pub = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
		odom_topic,
		10);

	m_joint_state_pub = node_ptr_->create_publisher<sensor_msgs::msg::JointState>(
		joint_state_topic,
		10);

	// Setup Swerve Model
	SwerveConfig swerve_model_config;
	swerve_model_config.max_wheel_lin_vel = 2.75 * M_PI * 650 / 60;
	swerve_model_config.module_type = swerve_type_e::DIFFERENTIAL;
	swerve_model_config.steering_ratio = 13.0 / 44.0;
	swerve_model_config.wheel_ratio = swerve_model_config.steering_ratio * 30.0 / 14.0;
	swerve_model_config.wheel_radius = 2.75 / 2.0;
	swerve_model_config.steering_kp = 0.18;
	swerve_model_config.max_wheel_actuator_vel = 650.0;

	swerve_model_config.module_positions["left_front"] = Eigen::Vector2d(0.1143, 0.1143);
	swerve_model_config.module_positions["right_front"] = Eigen::Vector2d(0.1143, -0.1143);
	swerve_model_config.module_positions["left_back"] = Eigen::Vector2d(-0.1143, 0.1143);
	swerve_model_config.module_positions["right_back"] = Eigen::Vector2d(-0.1143, -0.1143);

	m_swerve_model_ptr = std::make_shared<SwerveModel>(swerve_model_config);
}

void SwerveRobotPlugin::onNewSensorData(){
	auto module_jacobian = m_swerve_model_ptr->getModuleJacobian();

	std::unordered_map<std::string, std::tuple<std::string, std::string, std::string> > module_motor_mapping{
		{"left_front", std::tuple<std::string, std::string, std::string>("drive_flr", "drive_fll", "steering_front_left")},
		{"right_front", std::tuple<std::string, std::string, std::string>("drive_frr", "drive_frl", "steering_front_right")},
		{"left_back", std::tuple<std::string, std::string, std::string>("drive_blf", "drive_blb", "steering_back_left")},
		{"right_back", std::tuple<std::string, std::string, std::string>("drive_brf", "drive_brb", "steering_back_right")}
	};

	// Update each swerve module from new device data
	for(const auto& [module_name, device_name_tuple] : module_motor_mapping){
		// Get Device Names for this module
		std::string m1_name = std::get<0>(device_name_tuple);
		std::string m2_name = std::get<1>(device_name_tuple);
		std::string steering_name = std::get<2>(device_name_tuple);

		// Make a new swerve state to update the model
		ModuleState new_state;

		// Populate
		auto m1_position = rhi_ptr_->getMotorPosition(m1_name);
		auto m2_position = rhi_ptr_->getMotorPosition(m2_name);
		new_state.wheel_position = (module_jacobian * Eigen::Vector2d(m1_position, m2_position))[0];

		auto m1_velocity = rhi_ptr_->getMotorVelocityRPM(m1_name);
		auto m2_velocity = rhi_ptr_->getMotorVelocityRPM(m2_name);
		new_state.wheel_velocity = (module_jacobian * Eigen::Vector2d(m1_velocity, m2_velocity))[0];

		new_state.steering_angle = rhi_ptr_->getRotationSensorAngleDegrees(steering_name);
		new_state.steering_velocity = rhi_ptr_->getRotationSensorVelocityRPM(steering_name);

		m_swerve_model_ptr->setModuleState(module_name, new_state);
	}

	m_swerve_model_ptr->updateSwerveModel();

	publishSwerveVisualization();
}

void SwerveRobotPlugin::disabled(){
}

void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;
}
void SwerveRobotPlugin::teleop(double current_time){
	std::cout << "Teleop: " << current_time << std::endl;
	auto joy_data = rhi_ptr_->getMainJoystickData();

	m_swerve_model_ptr->calculateKinematicSwerveController(joy_data->right_x, joy_data->right_y, -joy_data->left_x);


	std::unordered_map<std::string, std::pair<std::string, std::string> > module_actuator_motor_mapping{
		{"left_front", std::pair<std::string, std::string>("drive_flr", "drive_fll")},
		{"right_front", std::pair<std::string, std::string>("drive_frr", "drive_frl")},
		{"left_back", std::pair<std::string, std::string>("drive_blf", "drive_blb")},
		{"right_back", std::pair<std::string, std::string>("drive_brf", "drive_brb")}
	};

	for(const auto & [module_name, motor_name_pair] : module_actuator_motor_mapping){
		std::string m1_name = motor_name_pair.first;
		std::string m2_name = motor_name_pair.second;
		auto command = m_swerve_model_ptr->getModuleCommand(module_name);
		rhi_ptr_->setMotorCurrentLimitMilliAmps(m1_name, 2500);
		rhi_ptr_->setMotorVelocityCommandRPM(m1_name, command.actuator_velocity_commands[0]);
		rhi_ptr_->setMotorVoltageCommandPercent(m1_name, command.actuator_voltage_commands[0]);

		rhi_ptr_->setMotorCurrentLimitMilliAmps(m2_name, 2500);
		rhi_ptr_->setMotorVelocityCommandRPM(m2_name, command.actuator_velocity_commands[1]);
		rhi_ptr_->setMotorVoltageCommandPercent(m2_name, command.actuator_voltage_commands[1]);
	}
}

void SwerveRobotPlugin::poseUpdateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
}

void SwerveRobotPlugin::publishSwerveVisualization(){
	std::unordered_map<std::string, std::pair<std::string, std::string> > joint_name_map{
		{"left_front", std::pair<std::string, std::string>("wheel_joint_front_left", "steering_joint_front_left")},
		{"right_front", std::pair<std::string, std::string>("wheel_joint_front_right", "steering_joint_front_right")},
		{"left_back", std::pair<std::string, std::string>("wheel_joint_back_left", "steering_joint_back_left")},
		{"right_back", std::pair<std::string, std::string>("wheel_joint_back_right", "steering_joint_back_right")},
	};
	auto msg = sensor_msgs::msg::JointState{};
	msg.header.stamp = node_ptr_->get_clock()->now();

	for(const auto & [module_name, joint_name_pair] : joint_name_map){
		auto wheel_joint_name = joint_name_pair.first;
		auto steering_joint_name = joint_name_pair.second;
		auto module_state = m_swerve_model_ptr->getCurrentModuleState(module_name);

		msg.name.push_back(wheel_joint_name);
		msg.position.push_back(module_state.wheel_position * M_PI / 180.0);
		msg.velocity.push_back(module_state.wheel_velocity * M_PI / 30.0);

		msg.name.push_back(steering_joint_name);
		msg.position.push_back(module_state.steering_angle * M_PI / 180.0);
		msg.velocity.push_back(module_state.steering_velocity * M_PI / 30.0);
	}

	m_joint_state_pub->publish(msg);
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)