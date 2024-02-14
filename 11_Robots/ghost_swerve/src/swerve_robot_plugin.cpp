#include <iostream>
#include <ghost_swerve/swerve_model.hpp>
#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <ghost_util/unit_conversion_utils.hpp>

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

	node_ptr_->declare_parameter("marker_array_topic", "/swerve_markers");
	std::string marker_array_topic = node_ptr_->get_parameter("marker_array_topic").as_string();

	node_ptr_->declare_parameter("swerve_robot_plugin.k1", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k2", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k3", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k4", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k5", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k6", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k7", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k8", 0.0);
	node_ptr_->declare_parameter("swerve_robot_plugin.k9", 0.0);
	m_k1 = node_ptr_->get_parameter("swerve_robot_plugin.k1").as_double();
	m_k2 = node_ptr_->get_parameter("swerve_robot_plugin.k2").as_double();
	m_k3 = node_ptr_->get_parameter("swerve_robot_plugin.k3").as_double();
	m_k4 = node_ptr_->get_parameter("swerve_robot_plugin.k4").as_double();
	m_k5 = node_ptr_->get_parameter("swerve_robot_plugin.k5").as_double();
	m_k6 = node_ptr_->get_parameter("swerve_robot_plugin.k6").as_double();
	m_k7 = node_ptr_->get_parameter("swerve_robot_plugin.k7").as_double();
	m_k8 = node_ptr_->get_parameter("swerve_robot_plugin.k8").as_double();
	m_k9 = node_ptr_->get_parameter("swerve_robot_plugin.k9").as_double();

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

	m_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
		marker_array_topic,
		10);

	// Setup Swerve Model
	SwerveConfig swerve_model_config;
	swerve_model_config.module_type = swerve_type_e::DIFFERENTIAL;
	swerve_model_config.steering_ratio = 13.0 / 44.0;
	swerve_model_config.wheel_ratio = swerve_model_config.steering_ratio * 30.0 / 14.0;
	swerve_model_config.wheel_radius = 2.75 / 2.0;
	swerve_model_config.steering_kp = 2.0;
	swerve_model_config.max_wheel_actuator_vel = 650.0;
	auto wheel_rad_per_sec = ghost_util::RPM_TO_RAD_PER_SEC * swerve_model_config.max_wheel_actuator_vel * swerve_model_config.wheel_ratio;
	swerve_model_config.max_wheel_lin_vel = wheel_rad_per_sec * swerve_model_config.wheel_radius * ghost_util::INCHES_TO_METERS;

	swerve_model_config.module_positions["left_front"] = Eigen::Vector2d(0.1143, 0.1143);
	swerve_model_config.module_positions["right_front"] = Eigen::Vector2d(0.1143, -0.1143);
	swerve_model_config.module_positions["left_back"] = Eigen::Vector2d(-0.1143, 0.1143);
	swerve_model_config.module_positions["right_back"] = Eigen::Vector2d(-0.1143, -0.1143);

	m_swerve_model_ptr = std::make_shared<SwerveModel>(swerve_model_config);
}

void SwerveRobotPlugin::onNewSensorData(){
	auto module_jacobian = m_swerve_model_ptr->getModuleJacobian();

	std::unordered_map<std::string, std::tuple<std::string, std::string, std::string> > module_motor_mapping{
		{"left_front", std::tuple<std::string, std::string, std::string>("drive_fll", "drive_flr", "steering_front_left")},
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

	publishOdometry();
	publishVisualization();
}

void SwerveRobotPlugin::disabled(){
}

void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;
}
void SwerveRobotPlugin::teleop(double current_time){
	auto joy_data = rhi_ptr_->getMainJoystickData();
	std::cout << "Teleop: " << current_time << std::endl;

	m_swerve_model_ptr->calculateKinematicSwerveController(joy_data->left_x / 127.0, joy_data->left_y / 127.0, -joy_data->right_x / 127.0);

	std::unordered_map<std::string, std::pair<std::string, std::string> > module_actuator_motor_mapping{
		{"left_front", std::pair<std::string, std::string>("drive_fll", "drive_flr")},
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

void SwerveRobotPlugin::publishOdometry(){
	Eigen::Vector2d odom_loc = m_swerve_model_ptr->getOdometryLocation();
	double odom_angle = m_swerve_model_ptr->getOdometryAngle();

	nav_msgs::msg::Odometry msg{};
	msg.header.frame_id = "odom";
	msg.header.stamp = node_ptr_->get_clock()->now();
	msg.child_frame_id = "base_link";

	msg.pose.pose.position.x = odom_loc.x();
	msg.pose.pose.position.y = odom_loc.y();
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation.x = 0.0;
	msg.pose.pose.orientation.y = 0.0;
	msg.pose.pose.orientation.z = sin(odom_angle * 0.5);
	msg.pose.pose.orientation.w = cos(odom_angle * 0.5);

	// covariance is row major form
	std::array<double, 36> covariance = {m_k1, m_k4, 0.0, m_k7, 0.0, 0.0,
		                             m_k2, m_k5, 0.0, m_k8, 0.0, 0.0,
		                             m_k3, m_k6, 0.0, m_k9, 0.0, 0.0};
	msg.pose.covariance = covariance;
	// msg.twist.twist.linear.x =
	// msg.twist.twist.linear.y
	// msg.twist.twist.linear.z = 0.0;
	// msg.twist.twist.angular.x = 0.0;
	// msg.twist.twist.angular.y = 0.0;
	// msg.twist.twist.angular.z

	m_odom_pub->publish(msg);
}

void SwerveRobotPlugin::publishVisualization(){
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

	std::vector<std::string> module_names{
		"left_front",
		"right_front",
		"left_back",
		"right_back"
	};

	visualization_msgs::msg::MarkerArray viz_msg;
	int j = 0;
	for(const auto& name : module_names){
		auto module_command = m_swerve_model_ptr->getModuleCommand(name);
		auto module_position = m_swerve_model_ptr->getConfig().module_positions.at(name);
		auto marker_msg = visualization_msgs::msg::Marker{};

		marker_msg.header.frame_id = "base_link";
		marker_msg.header.stamp = node_ptr_->get_clock()->now();
		marker_msg.id = j++;
		marker_msg.action = 0;
		marker_msg.type = 0;
		marker_msg.scale.x = 0.01;
		marker_msg.scale.y = 0.01;
		marker_msg.scale.z = 0.01;
		marker_msg.color.a = 1;

		geometry_msgs::msg::Point p0{};
		p0.x = module_position.x();
		p0.y = module_position.y();
		p0.z = 0.0;
		marker_msg.points.push_back(p0);
		geometry_msgs::msg::Point p1{};
		p1.x = module_position.x() + module_command.wheel_velocity_vector.x();
		p1.y = module_position.y() + module_command.wheel_velocity_vector.y();
		p1.z = 0.0;
		marker_msg.points.push_back(p1);

		viz_msg.markers.push_back(marker_msg);
	}
	m_viz_pub->publish(viz_msg);
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)