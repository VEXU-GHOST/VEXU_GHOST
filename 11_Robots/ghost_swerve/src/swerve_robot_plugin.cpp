#include <iostream>
#include <ghost_swerve/swerve_model.hpp>
#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

namespace ghost_swerve {

SwerveRobotPlugin::SwerveRobotPlugin(){
	m_digital_io = std::vector<bool>(8, false);
	m_digital_io_name_map = std::unordered_map<std::string, size_t>{
		{"claw", 0},
		{"tail", 1}};
}

void SwerveRobotPlugin::initialize(){
	std::cout << "Swerve Robot Initialization!" << std::endl;

	// node_ptr_->declare_parameter("trajectory_topic", "/motion_planning/trajectory");
	// std::string trajectory_topic = node_ptr_->get_parameter("trajectory_topic").as_string();

	node_ptr_->declare_parameter("odom_topic", "/sensors/wheel_odom");
	std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

	node_ptr_->declare_parameter("pose_topic", "/odometry/filtered");
	std::string pose_topic = node_ptr_->get_parameter("pose_topic").as_string();

	node_ptr_->declare_parameter("joint_state_topic", "/joint_states");
	std::string joint_state_topic = node_ptr_->get_parameter("joint_state_topic").as_string();

	node_ptr_->declare_parameter("marker_array_topic", "/swerve_markers");
	std::string marker_array_topic = node_ptr_->get_parameter("marker_array_topic").as_string();

	node_ptr_->declare_parameter("trajectory_marker_topic", "/trajectory_markers");
	std::string trajectory_marker_topic = node_ptr_->get_parameter("trajectory_marker_topic").as_string();

	node_ptr_->declare_parameter("swerve_robot_plugin.joy_angle_control_threshold", 0.0);
	m_joy_angle_control_threshold = node_ptr_->get_parameter("swerve_robot_plugin.joy_angle_control_threshold").as_double();

	node_ptr_->declare_parameter<std::string>("bt_path");
	std::string bt_path = node_ptr_->get_parameter("bt_path").as_string();

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

	node_ptr_->declare_parameter("swerve_robot_plugin.move_to_pose_kp_xy", 0.0);
	m_move_to_pose_kp_xy = node_ptr_->get_parameter("swerve_robot_plugin.move_to_pose_kp_xy").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.move_to_pose_kp_theta", 0.0);
	m_move_to_pose_kp_theta = node_ptr_->get_parameter("swerve_robot_plugin.move_to_pose_kp_theta").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.joystick_slew_rate", 0.0);
	m_joystick_slew_rate = node_ptr_->get_parameter("swerve_robot_plugin.joystick_slew_rate").as_double();

	// Setup Swerve Model
	SwerveConfig swerve_model_config;
	swerve_model_config.module_type = swerve_type_e::DIFFERENTIAL;
	swerve_model_config.steering_ratio = 13.0 / 44.0;
	swerve_model_config.wheel_ratio = swerve_model_config.steering_ratio * 30.0 / 14.0;
	swerve_model_config.wheel_radius = 2.75 / 2.0;

	node_ptr_->declare_parameter("swerve_robot_plugin.angle_control_kp", 0.2);
	swerve_model_config.angle_control_kp = node_ptr_->get_parameter("swerve_robot_plugin.angle_control_kp").as_double();

	swerve_model_config.move_to_pose_kp = m_move_to_pose_kp_xy;

	node_ptr_->declare_parameter("swerve_robot_plugin.steering_kp", 2.0);
	swerve_model_config.steering_kp = node_ptr_->get_parameter("swerve_robot_plugin.steering_kp").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.velocity_scaling_ratio", 1.0);
	swerve_model_config.velocity_scaling_ratio = node_ptr_->get_parameter("swerve_robot_plugin.velocity_scaling_ratio").as_double();
	node_ptr_->declare_parameter("swerve_robot_plugin.velocity_scaling_threshold", 0.7);
	swerve_model_config.velocity_scaling_threshold = node_ptr_->get_parameter("swerve_robot_plugin.velocity_scaling_threshold").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.lift_gear_ratio", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.lift_up_angle_deg", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.lift_climbed_angle_deg", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.lift_kP", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.lift_speed", 1.);
	double gear_ratio = node_ptr_->get_parameter("swerve_robot_plugin.lift_gear_ratio").as_double();
	swerve_model_config.lift_up_angle = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.lift_up_angle_deg").as_double();
	swerve_model_config.lift_climbed_angle = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.lift_climbed_angle_deg").as_double();
	swerve_model_config.lift_kP = node_ptr_->get_parameter("swerve_robot_plugin.lift_kP").as_double();
	swerve_model_config.lift_speed = node_ptr_->get_parameter("swerve_robot_plugin.lift_speed").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.stick_gear_ratio", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_upright_angle_deg", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_angle_skills", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_angle_normal", 1.);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_angle_soft_limit_offset", 1.);
	gear_ratio = node_ptr_->get_parameter("swerve_robot_plugin.stick_gear_ratio").as_double();

	swerve_model_config.stick_upright_angle = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_upright_angle_deg").as_double();
	swerve_model_config.stick_angle_skills = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_angle_skills").as_double();
	swerve_model_config.stick_angle_normal = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_angle_normal").as_double();
	swerve_model_config.stick_turn_offset = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_angle_soft_limit_offset").as_double();

	swerve_model_config.max_wheel_actuator_vel = 625.0;
	auto wheel_rad_per_sec = ghost_util::RPM_TO_RAD_PER_SEC * swerve_model_config.max_wheel_actuator_vel * swerve_model_config.wheel_ratio;
	swerve_model_config.max_wheel_lin_vel = wheel_rad_per_sec * swerve_model_config.wheel_radius * ghost_util::INCHES_TO_METERS;

	node_ptr_->declare_parameter("swerve_robot_plugin.max_ang_vel_slew", 0.); // 0 should stop the robot from moving when the param is not set
	swerve_model_config.max_ang_vel_slew = node_ptr_->get_parameter("swerve_robot_plugin.max_ang_vel_slew").as_double();
	node_ptr_->declare_parameter("swerve_robot_plugin.max_lin_vel_slew", 0.);
	swerve_model_config.max_lin_vel_slew = node_ptr_->get_parameter("swerve_robot_plugin.max_lin_vel_slew").as_double();

	swerve_model_config.module_positions["left_front"] = Eigen::Vector2d(0.15875, 0.15875);
	swerve_model_config.module_positions["right_front"] = Eigen::Vector2d(0.15875, -0.15875);
	swerve_model_config.module_positions["left_back"] = Eigen::Vector2d(-0.15875, 0.15875);
	swerve_model_config.module_positions["right_back"] = Eigen::Vector2d(-0.15875, -0.15875);

	m_swerve_model_ptr = std::make_shared<SwerveModel>(swerve_model_config);
	m_swerve_model_ptr->setFieldOrientedControl(true);

	// ROS Topics
	m_robot_pose_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
		pose_topic,
		10,
		std::bind(&SwerveRobotPlugin::poseUpdateCallback, this, _1));

	m_odom_pub = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
		odom_topic,
		10);

	m_joint_state_pub = node_ptr_->create_publisher<sensor_msgs::msg::JointState>(
		joint_state_topic,
		10);

	m_swerve_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
		marker_array_topic,
		10);

	m_trajectory_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
		trajectory_marker_topic,
		10);

	bt_ = std::make_shared<SwerveTree>(bt_path, rhi_ptr_, m_swerve_model_ptr);

	m_start_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StartRecorder>(
		"bag_recorder/start");

	m_stop_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StopRecorder>(
		"bag_recorder/stop");

	imu_pub = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(
		"/sensors/imu",
		10);
}

void SwerveRobotPlugin::onNewSensorData(){
	auto module_jacobian = m_swerve_model_ptr->getModuleJacobian();

	std::unordered_map<std::string, std::tuple<std::string, std::string, std::string> > module_motor_mapping{
		{"left_front", std::tuple<std::string, std::string, std::string>("drive_fll", "drive_flr", "steering_front_left")},
		{"right_front", std::tuple<std::string, std::string, std::string>("drive_frr", "drive_frl", "steering_front_right")},
		{"left_back", std::tuple<std::string, std::string, std::string>("drive_bll", "drive_blr", "steering_back_left")},
		{"right_back", std::tuple<std::string, std::string, std::string>("drive_brr", "drive_brl", "steering_back_right")}};

	// Update each swerve module from new device data
	for(const auto &[module_name, device_name_tuple] : module_motor_mapping){
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


	sensor_msgs::msg::Imu imu_msg{};
	imu_msg.header.frame_id = "imu_link";
	imu_msg.header.stamp = node_ptr_->get_clock()->now();
	imu_msg.linear_acceleration.x = rhi_ptr_->getInertialSensorXAccel("imu");
	imu_msg.linear_acceleration.y = rhi_ptr_->getInertialSensorYAccel("imu");
	imu_msg.linear_acceleration.z = rhi_ptr_->getInertialSensorZAccel("imu");
	imu_msg.angular_velocity.x = rhi_ptr_->getInertialSensorXRate("imu") * ghost_util::DEG_TO_RAD;
	imu_msg.angular_velocity.y = rhi_ptr_->getInertialSensorYRate("imu") * ghost_util::DEG_TO_RAD;
	imu_msg.angular_velocity.z = rhi_ptr_->getInertialSensorZRate("imu") * ghost_util::DEG_TO_RAD;
	imu_pub->publish(imu_msg);

	m_swerve_model_ptr->updateSwerveModel();

	publishOdometry();
	publishVisualization();
	// publishTrajectoryVisualization();
}

void SwerveRobotPlugin::disabled(){
}

void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;

	// bt_->tick_tree();
	publishTrajectoryVisualization();

	auto command_map = get_commands(current_time);
	double des_pos_x = (command_map.count("x_pos") != 0) ? command_map.at("x_pos") : 0.0;
	double des_vel_x = (command_map.count("x_vel") != 0) ? command_map.at("x_vel") : 0.0;
	double des_pos_y = (command_map.count("y_pos") != 0) ? command_map.at("y_pos") : 0.0;
	double des_vel_y = (command_map.count("y_vel") != 0) ? command_map.at("y_vel") : 0.0;
	double des_theta = (command_map.count("angle_pos") != 0) ? command_map.at("angle_pos") : 0.0;
	double des_theta_vel = (command_map.count("angle_vel") != 0) ? command_map.at("angle_vel") : 0.0;
	double pos_threshold = (command_map.count("threshold_pos") != 0) ? command_map.at("threshold_pos") : 0.0;
	double theta_threshold = (command_map.count("threshold_vel") != 0) ? command_map.at("threshold_vel") : 0.0;

	// Get best state estimate
	auto curr_location = m_swerve_model_ptr->getWorldLocation();
	double curr_theta = m_swerve_model_ptr->getWorldAngleRad();

	// Calculate velocity command from motion plan
	double x_error = des_pos_x - curr_location.x();
	double y_error = des_pos_y - curr_location.y();
	double theta_error = ghost_util::SmallestAngleDistRad(des_theta, curr_theta);
	double vel_cmd_x = (abs(x_error) <= pos_threshold / 2.0) ? 0.0 : des_vel_x + (x_error) * m_move_to_pose_kp_xy;
	double vel_cmd_y = (abs(y_error) <= pos_threshold / 2.0) ? 0.0 : des_vel_y + (y_error) * m_move_to_pose_kp_xy;
	double vel_cmd_theta = (abs(theta_error) <= theta_threshold) ? 0.0 : des_theta_vel + theta_error * m_move_to_pose_kp_theta;

	std::cout << "pos_threshold: " << pos_threshold << std::endl;
	std::cout << "theta_threshold: " << theta_threshold << std::endl;


	std::cout << "des_pos_x: " << des_pos_x << std::endl;
	std::cout << "des_pos_y: " << des_pos_y << std::endl;
	std::cout << "x_error: " << x_error << std::endl;
	std::cout << "y_error: " << y_error << std::endl;

	std::cout << "vel cmd x: " << vel_cmd_x << std::endl;
	std::cout << "vel cmd y: " << vel_cmd_y << std::endl;

	// calculateKinematicSwerveControllerVelocity(right_cmd * m_max_base_lin_vel, forward_cmd * m_max_base_lin_vel, clockwise_cmd * m_max_base_ang_vel);
	m_swerve_model_ptr->calculateKinematicSwerveControllerVelocity(-vel_cmd_y, vel_cmd_x, -vel_cmd_theta);
	// m_swerve_model_ptr->calculateKinematicSwerveControllerAngleControl(vel_cmd_x, vel_cmd_y, des_theta);

	updateDrivetrainMotors();
}

// sorry for puutting this here ik its kinda gross
float tempPID(std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_, const std::string &motor1, const std::string &motor2, float pos_want, double kP){
	float pos1 = rhi_ptr_->getMotorPosition(motor1);
	float pos2 = rhi_ptr_->getMotorPosition(motor2);
	float pos = (pos1 + pos2) / 2;
	float action = std::clamp((pos_want - pos) * kP, -100., 100.); // TODO ???
	if(fabs(action) < 1.5){
		action = 0;
	}
	rhi_ptr_->setMotorVoltageCommandPercent(motor1, action);
	rhi_ptr_->setMotorVoltageCommandPercent(motor2, action);
	// std::cout << "pos1: " << pos1 << " pos2: " << pos2 << " want: " << pos_want << " kP " << kP << " error " << (pos_want - pos) << " action " << action << std::endl;
	return pos - pos_want;
}

void SwerveRobotPlugin::teleop(double current_time){
	auto joy_data = rhi_ptr_->getMainJoystickData();
	// std::cout << "Teleop: " << current_time << std::endl;

	if(joy_data->btn_u){
		if(!m_auton_button_pressed){
			m_auton_button_pressed = true;

			//   Odometry or whatever for auton
			m_last_odom_angle = ghost_util::DEG_TO_RAD * 0.0;
			m_curr_odom_angle = m_last_odom_angle;
			m_curr_odom_loc.x() = ghost_util::INCHES_TO_METERS * 0.0;
			m_curr_odom_loc.y() = ghost_util::INCHES_TO_METERS * 0.0;
			m_last_odom_loc.x() = m_curr_odom_loc.x();
			m_last_odom_loc.y() = m_last_odom_loc.y();
			m_swerve_model_ptr->setWorldPose(m_curr_odom_loc.x(), m_curr_odom_loc.y());
			m_swerve_model_ptr->setWorldAngle(m_curr_odom_angle);
		}
		autonomous(current_time - m_auton_start_time);
	}
	else{
		// Reset Auton Tester
		m_auton_start_time = current_time;
		m_auton_button_pressed = false;
		m_auton_index = 0;

		// Toggle Field vs Robot Oriented
		if(joy_data->btn_x && !m_toggle_swerve_field_control_btn_pressed){
			m_swerve_model_ptr->setFieldOrientedControl(!m_swerve_model_ptr->isFieldOrientedControl());
			m_toggle_swerve_field_control_btn_pressed = true;
		}
		else if(!joy_data->btn_x){
			m_toggle_swerve_field_control_btn_pressed = false;
		}

		if(joy_data->btn_l){
			m_last_odom_angle = 0.0;
			m_curr_odom_angle = 0.0;
			m_swerve_model_ptr->setOdometryAngle(0.0);
		}
		// Toggle Bag Recorder
		if(joy_data->btn_y && !m_recording_btn_pressed){
			m_recording_btn_pressed = true;

			if(!m_recording){
				auto req = std::make_shared<ghost_msgs::srv::StartRecorder::Request>();
				m_start_recorder_client->async_send_request(req);
			}
			else{
				auto req = std::make_shared<ghost_msgs::srv::StopRecorder::Request>();
				m_stop_recorder_client->async_send_request(req);
			}

			m_recording = !m_recording;
		}
		else if(!joy_data->btn_y){
			m_recording_btn_pressed = false;
		}

		// if(m_swerve_angle_control){
		// 	if(Eigen::Vector2d(joy_data->right_y / 127.0, joy_data->right_x / 127.0).norm() > m_joy_angle_control_threshold){
		// 		m_angle_target = atan2(joy_data->right_y / 127.0, joy_data->right_x / 127.0) - M_PI / 2;
		// 	}

		// 	m_swerve_model_ptr->calculateKinematicSwerveControllerAngleControl(joy_data->left_x, joy_data->left_y, m_angle_target);
		// }
		// else{
		double scale = (joy_data->btn_r1) ? 0.5 : 1.0;

		m_curr_x_cmd = joy_data->left_x / 127.0 * scale;
		m_curr_y_cmd = joy_data->left_y / 127.0 * scale;
		m_curr_theta_cmd = joy_data->right_x / 127.0 * scale;

		m_swerve_model_ptr->calculateKinematicSwerveControllerNormalized(m_curr_x_cmd, m_curr_y_cmd, m_curr_theta_cmd);
		m_angle_target = m_swerve_model_ptr->getWorldAngleRad();
		// }

		m_last_x_cmd = m_curr_x_cmd;
		m_last_y_cmd = m_curr_y_cmd;
		m_last_theta_cmd = m_curr_theta_cmd;

		updateDrivetrainMotors();

		// Intake
		if(joy_data->btn_r1){
			rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
			rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", -1.0);
		}
		else if(joy_data->btn_r2){
			rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
			rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", 1.0);
		}
		else{
			rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
			rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", 0);
		}

		// Climb Testing
		if(joy_data->btn_l1){
			rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 2500);
			rhi_ptr_->setMotorVoltageCommandPercent("lift_left", -1.0);

			rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 2500);
			rhi_ptr_->setMotorVoltageCommandPercent("lift_right", -1.0);
		}
		else if(joy_data->btn_l2){
			rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 2500);
			rhi_ptr_->setMotorVoltageCommandPercent("lift_left", 1.0);

			rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 2500);
			rhi_ptr_->setMotorVoltageCommandPercent("lift_right", 1.0);
		}
		else{
			rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 0);
			rhi_ptr_->setMotorVoltageCommandPercent("lift_left", 0);

			rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 0);
			rhi_ptr_->setMotorVoltageCommandPercent("lift_right", 0);
		}

		rhi_ptr_->setDigitalIO(m_digital_io);
	}
}

void SwerveRobotPlugin::updateDrivetrainMotors(){
	std::unordered_map<std::string, std::pair<std::string, std::string> > module_actuator_motor_mapping{
		{"left_front", std::pair<std::string, std::string>("drive_fll", "drive_flr")},
		{"right_front", std::pair<std::string, std::string>("drive_frr", "drive_frl")},
		{"left_back", std::pair<std::string, std::string>("drive_bll", "drive_blr")},
		{"right_back", std::pair<std::string, std::string>("drive_brr", "drive_brl")}};

	for(const auto &[module_name, motor_name_pair] : module_actuator_motor_mapping){
		std::string m1_name = motor_name_pair.first;
		std::string m2_name = motor_name_pair.second;
		auto command = m_swerve_model_ptr->getModuleCommand(module_name);

		if(m_climb_mode && !m_claw_open){
			// reduce current to drivetrain when in climb mode and hooked onto the pole
			rhi_ptr_->setMotorCurrentLimitMilliAmps(m1_name, 750);
			rhi_ptr_->setMotorCurrentLimitMilliAmps(m2_name, 750);
		}
		else{
			rhi_ptr_->setMotorCurrentLimitMilliAmps(m1_name, 2500);
			rhi_ptr_->setMotorCurrentLimitMilliAmps(m2_name, 2500);
		}

		rhi_ptr_->setMotorVelocityCommandRPM(m1_name, command.actuator_velocity_commands[0]);
		rhi_ptr_->setMotorVoltageCommandPercent(m1_name, command.actuator_voltage_commands[0]);

		rhi_ptr_->setMotorVelocityCommandRPM(m2_name, command.actuator_velocity_commands[1]);
		rhi_ptr_->setMotorVoltageCommandPercent(m2_name, command.actuator_voltage_commands[1]);
	}
}


void SwerveRobotPlugin::poseUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
	double theta = ghost_util::quaternionToYawRad(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	m_swerve_model_ptr->setWorldPose(msg->pose.pose.position.x, msg->pose.pose.position.y);
	m_swerve_model_ptr->setWorldAngle(theta);
}

void SwerveRobotPlugin::publishOdometry(){
	m_curr_odom_loc = m_swerve_model_ptr->getOdometryLocation();
	m_curr_odom_angle = m_swerve_model_ptr->getOdometryAngle();

	nav_msgs::msg::Odometry msg{};
	msg.header.frame_id = "odom";
	msg.header.stamp = node_ptr_->get_clock()->now();
	msg.child_frame_id = "base_link";

	msg.pose.pose.position.x = m_curr_odom_loc.x();
	msg.pose.pose.position.y = m_curr_odom_loc.y();
	msg.pose.pose.position.z = 0.0;
	ghost_util::yawToQuaternionRad(
		m_curr_odom_angle,
		msg.pose.pose.orientation.w,
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z);

	// Calculate differences for odometry
	auto odom_diff_x = std::fabs(m_curr_odom_loc.x() - m_last_odom_loc.x());
	auto odom_diff_y = std::fabs(m_curr_odom_loc.y() - m_last_odom_loc.y());
	auto odom_diff_theta = std::fabs(ghost_util::SmallestAngleDistRad(m_curr_odom_angle, m_last_odom_angle));

	// Holonomic Motion Model
	Eigen::Vector3d diff_std = Eigen::Vector3d(
		m_k1 * odom_diff_x + m_k2 * odom_diff_y + m_k3 * odom_diff_theta,
		m_k4 * odom_diff_x + m_k5 * odom_diff_y + m_k6 * odom_diff_theta,
		m_k7 * odom_diff_x + m_k8 * odom_diff_y + m_k9 * odom_diff_theta);

	m_curr_odom_std += diff_std;
	m_curr_odom_cov = m_curr_odom_std.array().square();

	// covariance is row major form
	std::array<double, 36> pose_covariance{
		m_curr_odom_cov.x(), 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, m_curr_odom_cov.y(), 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, m_curr_odom_cov.z()};

	msg.pose.covariance = pose_covariance;

	auto current_velocity = m_swerve_model_ptr->getBaseVelocityCurrent();

	msg.twist.twist.linear.x = current_velocity.x();
	msg.twist.twist.linear.y = current_velocity.y();
	msg.twist.twist.linear.z = 0.0;
	msg.twist.twist.angular.x = 0.0;
	msg.twist.twist.angular.y = 0.0;
	msg.twist.twist.angular.z = current_velocity.z();

	double sigma_x_vel =
		m_k1 * current_velocity.x() +
		m_k2 * current_velocity.y() +
		m_k3 * abs(current_velocity.z());
	double sigma_y_vel =
		m_k4 * current_velocity.x() +
		m_k5 * current_velocity.y() +
		m_k6 * abs(current_velocity.z());
	// Get noisy angle
	double sigma_tht_vel =
		m_k7 * current_velocity.x() +
		m_k8 * current_velocity.y() +
		m_k9 * abs(current_velocity.z());

	std::array<double, 36> vel_covariance{
		sigma_x_vel * sigma_x_vel, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, sigma_y_vel * sigma_y_vel, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, sigma_tht_vel * sigma_tht_vel};

	msg.twist.covariance = vel_covariance;

	m_odom_pub->publish(msg);

	m_last_odom_loc = m_curr_odom_loc;
	m_last_odom_angle = m_curr_odom_angle;
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

	for(const auto &[module_name, joint_name_pair] : joint_name_map){
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
		"right_back"};

	visualization_msgs::msg::MarkerArray viz_msg;
	int j = 0;
	for(const auto &name : module_names){
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

	auto curr_vel_marker_msg = visualization_msgs::msg::Marker{};
	curr_vel_marker_msg.header.frame_id = "base_link";
	curr_vel_marker_msg.header.stamp = node_ptr_->get_clock()->now();
	curr_vel_marker_msg.id = j++;
	curr_vel_marker_msg.action = 0;
	curr_vel_marker_msg.type = 0;
	curr_vel_marker_msg.scale.x = 0.01;
	curr_vel_marker_msg.scale.y = 0.01;
	curr_vel_marker_msg.scale.z = 0.01;
	curr_vel_marker_msg.color.b = 1.0;
	curr_vel_marker_msg.color.a = 1;

	curr_vel_marker_msg.points.push_back(geometry_msgs::msg::Point{});
	auto curr_base_vel = m_swerve_model_ptr->getBaseVelocityCurrent();

	geometry_msgs::msg::Point p1_curr{};
	p1_curr.x = curr_base_vel.x();
	p1_curr.y = curr_base_vel.y();
	curr_vel_marker_msg.points.push_back(p1_curr);

	viz_msg.markers.push_back(curr_vel_marker_msg);

	auto cmd_vel_marker_msg = visualization_msgs::msg::Marker{};
	cmd_vel_marker_msg.header.frame_id = "base_link";
	cmd_vel_marker_msg.header.stamp = node_ptr_->get_clock()->now();
	cmd_vel_marker_msg.id = j++;
	cmd_vel_marker_msg.action = 0;
	cmd_vel_marker_msg.type = 0;
	cmd_vel_marker_msg.scale.x = 0.01;
	cmd_vel_marker_msg.scale.y = 0.01;
	cmd_vel_marker_msg.scale.z = 0.01;
	cmd_vel_marker_msg.color.r = 1.0;
	cmd_vel_marker_msg.color.a = 1;

	cmd_vel_marker_msg.points.push_back(geometry_msgs::msg::Point{});
	auto cmd_base_vel = m_swerve_model_ptr->getBaseVelocityCommand();

	geometry_msgs::msg::Point p1_cmd{};
	p1_cmd.x = cmd_base_vel.x();
	p1_cmd.y = cmd_base_vel.y();
	cmd_vel_marker_msg.points.push_back(p1_cmd);

	viz_msg.markers.push_back(cmd_vel_marker_msg);

	m_swerve_viz_pub->publish(viz_msg);
}

void SwerveRobotPlugin::publishTrajectoryVisualization(){
	visualization_msgs::msg::MarkerArray viz_msg;
	auto time = trajectory_motor_map_["x"].time_vector;

	// RCLCPP_INFO(node_ptr_->get_logger(), "publishing trajectory viz");
	if(trajectory_motor_map_.size() == 0){
		RCLCPP_WARN(node_ptr_->get_logger(), "empty trajectory");
		return;
	}
	if(trajectory_motor_map_["x"].time_vector.size() == 0){
		RCLCPP_WARN(node_ptr_->get_logger(), "empty time vector");
		return;
	}
	auto x = trajectory_motor_map_["x"].position_vector;
	auto x_vel = trajectory_motor_map_["x"].velocity_vector;
	auto y = trajectory_motor_map_["y"].position_vector;
	auto y_vel = trajectory_motor_map_["y"].velocity_vector;
	auto ang = trajectory_motor_map_["angle"].position_vector;
	auto ang_vel = trajectory_motor_map_["angle"].velocity_vector;
	int j = 30;

	for(int i = 0; i < trajectory_motor_map_["x"].time_vector.size(); i += 50){
		auto marker_msg = visualization_msgs::msg::Marker{};

		marker_msg.header.frame_id = "odom";
		marker_msg.header.stamp = node_ptr_->get_clock()->now();
		marker_msg.id = j++;
		marker_msg.action = 0;
		marker_msg.type = 0;
		double vel = sqrt(x_vel[i] * x_vel[i] + y_vel[i] * y_vel[i]);
		marker_msg.scale.x = vel;
		marker_msg.scale.y = 0.1;
		marker_msg.scale.z = 0.1;
		marker_msg.pose.position.x = x[i];
		marker_msg.pose.position.y = y[i];
		marker_msg.pose.position.z = 0;
		double w, x, y, z;
		ghost_util::yawToQuaternionRad(ang[i], w, x, y, z);
		marker_msg.pose.orientation.w = w;
		marker_msg.pose.orientation.x = x;
		marker_msg.pose.orientation.y = y;
		marker_msg.pose.orientation.z = z;
		marker_msg.color.a = 1;
		marker_msg.color.r = 1 - time[i] / time[time.size() - 1];
		marker_msg.color.g = 0;
		marker_msg.color.b = time[i] / time[time.size() - 1];

		viz_msg.markers.push_back(marker_msg);
	}

	// RCLCPP_INFO(node_ptr_->get_logger(), "publishing trajectory arrows");

	m_trajectory_viz_pub->publish(viz_msg);
}

} // namespace ghost_swerve

PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)