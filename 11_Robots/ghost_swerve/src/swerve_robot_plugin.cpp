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
		{"right_wing", 1},
		{"left_wing", 2},
		{"tail", 3}
	};
}

void SwerveRobotPlugin::initialize(){
	std::cout << "Swerve Robot Initialization!" << std::endl;

	// node_ptr_->declare_parameter("trajectory_topic", "/motion_planning/trajectory");
	// std::string trajectory_topic = node_ptr_->get_parameter("trajectory_topic").as_string();

	node_ptr_->declare_parameter("odom_topic", "/sensors/wheel_odom");
	std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

	node_ptr_->declare_parameter("pose_topic", "/estimation/robot_pose");
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

	node_ptr_->declare_parameter("swerve_robot_plugin.move_to_pose_kp_x", 0.0);
	m_move_to_pose_kp_x = node_ptr_->get_parameter("swerve_robot_plugin.move_to_pose_kp_x").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.move_to_pose_kp_y", 0.0);
	m_move_to_pose_kp_y = node_ptr_->get_parameter("swerve_robot_plugin.move_to_pose_kp_y").as_double();

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

	node_ptr_->declare_parameter("swerve_robot_plugin.steering_kp", 2.0);
	swerve_model_config.steering_kp = node_ptr_->get_parameter("swerve_robot_plugin.steering_kp").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.velocity_scaling_ratio", 1.0);
	swerve_model_config.velocity_scaling_ratio = node_ptr_->get_parameter("swerve_robot_plugin.velocity_scaling_ratio").as_double();
	node_ptr_->declare_parameter("swerve_robot_plugin.velocity_scaling_threshold", 0.7);
	swerve_model_config.velocity_scaling_threshold = node_ptr_->get_parameter("swerve_robot_plugin.velocity_scaling_threshold").as_double();

	node_ptr_->declare_parameter("swerve_robot_plugin.lift_gear_ratio", NULL);
	node_ptr_->declare_parameter("swerve_robot_plugin.lift_up_angle_deg", NULL);
	node_ptr_->declare_parameter("swerve_robot_plugin.lift_up_speed_degps", NULL);
	double gear_ratio = node_ptr_->get_parameter("swerve_robot_plugin.lift_gear_ratio").as_double();
	swerve_model_config.lift_up_angle = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.lift_up_angle_deg").as_double();
	#define DPSTORPM(x) (x  / 360.)  * 60.
	// this really should be somewhere better
	swerve_model_config.lift_speed_rpm = DPSTORPM(gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.lift_up_speed_degps").as_double());

	node_ptr_->declare_parameter("swerve_robot_plugin.stick_gear_ratio", NULL);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_upright_angle_deg", NULL);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_endpoint1_deg", NULL);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_endpoint2_deg", NULL);
	node_ptr_->declare_parameter("swerve_robot_plugin.stick_angle_soft_limit_offset", NULL);
	gear_ratio = node_ptr_->get_parameter("swerve_robot_plugin.stick_gear_ratio").as_double();


	swerve_model_config.stick_upright_angle = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_upright_angle_deg").as_double();
	double endpoint1 = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_endpoint1_deg").as_double();
	double endpoint2 = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_endpoint2_deg").as_double();
	swerve_model_config.stick_angle_min = std::min(endpoint1, endpoint2);
	swerve_model_config.stick_angle_max = std::max(endpoint1, endpoint2);
	swerve_model_config.stick_turn_offset = gear_ratio * node_ptr_->get_parameter("swerve_robot_plugin.stick_angle_soft_limit_offset").as_double();


	swerve_model_config.max_wheel_actuator_vel = 625.0;
	auto wheel_rad_per_sec = ghost_util::RPM_TO_RAD_PER_SEC * swerve_model_config.max_wheel_actuator_vel * swerve_model_config.wheel_ratio;
	swerve_model_config.max_wheel_lin_vel = wheel_rad_per_sec * swerve_model_config.wheel_radius * ghost_util::INCHES_TO_METERS;

	swerve_model_config.module_positions["left_front"] = Eigen::Vector2d(0.1143, 0.1143);
	swerve_model_config.module_positions["right_front"] = Eigen::Vector2d(0.1143, -0.1143);
	swerve_model_config.module_positions["left_back"] = Eigen::Vector2d(-0.1143, 0.1143);
	swerve_model_config.module_positions["right_back"] = Eigen::Vector2d(-0.1143, -0.1143);

	m_swerve_model_ptr = std::make_shared<SwerveModel>(swerve_model_config);
	m_swerve_model_ptr->setFieldOrientedControl(true);

	// ROS Topics
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

	m_swerve_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
		marker_array_topic,
		10);

	m_trajectory_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
		trajectory_marker_topic,
		10);

	bt_ = std::make_shared<RunTree>(bt_path, rhi_ptr_);
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

	bt_->tick_tree();

	auto command_map = get_commands(current_time);
	double des_pos_x = (command_map.count("x_pos") != 0) ? command_map.at("x_pos") : 0.0;
	double des_vel_x = (command_map.count("x_vel") != 0) ? command_map.at("x_vel") : 0.0;
	double des_pos_y = (command_map.count("y_pos") != 0) ? command_map.at("y_pos") : 0.0;
	double des_vel_y = (command_map.count("y_vel") != 0) ? command_map.at("y_vel") : 0.0;
	double des_theta = (command_map.count("angle_pos") != 0) ? command_map.at("angle_pos") : 0.0;
	double des_theta_vel = (command_map.count("angle_vel") != 0) ? command_map.at("angle_vel") : 0.0;


	// Get best state estimate
	auto curr_location = m_swerve_model_ptr->getWorldLocation();
	double curr_theta = m_swerve_model_ptr->getWorldAngleRad();

	// Calculate velocity command from motion plan
	double vel_cmd_x = des_vel_x + (des_pos_x - curr_location.x()) * m_move_to_pose_kp_x;
	double vel_cmd_y = des_vel_y + (des_pos_y - curr_location.y()) * m_move_to_pose_kp_y;
	double vel_cmd_theta = des_theta_vel + ghost_util::SmallestAngleDistRad(des_theta, curr_theta) * m_move_to_pose_kp_theta;

	std::cout << "vel cmd x: " << vel_cmd_x << std::endl;

	// calculateKinematicSwerveControllerVelocity(right_cmd * m_max_base_lin_vel, forward_cmd * m_max_base_lin_vel, clockwise_cmd * m_max_base_ang_vel);

	m_swerve_model_ptr->calculateKinematicSwerveControllerVelocity(vel_cmd_x, vel_cmd_y, -vel_cmd_theta);

	updateDrivetrainMotors();
}
void SwerveRobotPlugin::teleop(double current_time){
	auto joy_data = rhi_ptr_->getMainJoystickData();
	std::cout << "Teleop: " << current_time << std::endl;

	if(joy_data->btn_u){
		autonomous(current_time - m_auton_start_time);
	}
	else{
		// Reset Auton Tester
		m_auton_start_time = current_time;

		// Toggle Field vs Robot Oriented
		if(joy_data->btn_x && !m_toggle_swerve_field_control_btn_pressed){
			m_swerve_model_ptr->setFieldOrientedControl(!m_swerve_model_ptr->isFieldOrientedControl());
			m_toggle_swerve_field_control_btn_pressed = true;
		}
		else if(!joy_data->btn_x){
			m_toggle_swerve_field_control_btn_pressed = false;
		}

		// Toggle Field vs Robot Oriented
		if(joy_data->btn_r && !m_toggle_swerve_angle_control_btn_pressed){
			m_swerve_angle_control = !m_swerve_angle_control;
			m_toggle_swerve_angle_control_btn_pressed = true;
		}
		else if(!joy_data->btn_r){
			m_toggle_swerve_angle_control_btn_pressed = false;
		}


		if(m_swerve_angle_control){
			if(Eigen::Vector2d(joy_data->right_y / 127.0, joy_data->right_x / 127.0).norm() > m_joy_angle_control_threshold){
				m_angle_target = atan2(joy_data->right_y / 127.0, joy_data->right_x / 127.0) - M_PI / 2;
			}

			m_swerve_model_ptr->calculateKinematicSwerveControllerAngleControl(joy_data->left_x, joy_data->left_y, m_angle_target);
		}
		else{
			double scale = (joy_data->btn_r1) ? 0.5 : 1.0;
			    << << << < HEAD

			    m_curr_x_cmd = joy_data->left_x / 127.0 * scale;
			m_curr_y_cmd = joy_data->left_y / 127.0 * scale;
			m_curr_theta_cmd = joy_data->right_x / 127.0 * scale;

			m_swerve_model_ptr->calculateKinematicSwerveControllerNormalized(m_curr_x_cmd, m_curr_y_cmd, m_curr_theta_cmd);
			== == == =
				m_swerve_model_ptr->calculateKinematicSwerveControllerJoystick(joy_data->left_x * scale, joy_data->left_y * scale, joy_data->right_x * scale);
			>> >> >> > 581927a62506ffdfd05c02b44c537e496229ca3d
			m_angle_target = m_swerve_model_ptr->getWorldAngleRad();
		}

		m_last_x_cmd = m_curr_x_cmd;
		m_last_y_cmd = m_curr_y_cmd;
		m_last_theta_cmd = m_curr_theta_cmd;

		updateDrivetrainMotors();

		// Set Wings
		m_digital_io[m_digital_io_name_map.at("right_wing")] = joy_data->btn_r2;
		m_digital_io[m_digital_io_name_map.at("left_wing")] = joy_data->btn_l2;


		// Toggle Climb Mode
		if(joy_data->btn_a && !m_climb_mode_btn_pressed){
			m_climb_mode = !m_climb_mode;
			// only on first toggle
			if(m_climb_mode){
				rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 2500);
				rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 2500);
				m_claw_open = true;
				rhi_ptr_->setMotorPositionCommand("lift_right", m_swerve_model_ptr->getConfig().lift_up_angle); // TODO MAXX IS THIS GOOD PRACTICE IDK
				rhi_ptr_->setMotorPositionCommand("lift_left", m_swerve_model_ptr->getConfig().lift_up_angle);
			}
			else{
				rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 0);
				rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 0);
				m_claw_open = false;
				rhi_ptr_->setMotorPositionCommand("lift_right",0);
				rhi_ptr_->setMotorPositionCommand("lift_left",0);
			}
			m_climb_mode_btn_pressed = true;
		}
		else if(!joy_data->btn_a){
			m_climb_mode_btn_pressed = false;
		}
		// Toggle Claw
		if(m_climb_mode){
			if(joy_data->btn_r2){
				m_claw_open = !m_claw_open;
				m_claw_btn_pressed = true;
			}

			if(joy_data->btn_l1){
				// degrees p s
				rhi_ptr_->setMotorVelocityCommandRPM("lift_right", m_swerve_model_ptr->getConfig().lift_speed_rpm);
				rhi_ptr_->setMotorVelocityCommandRPM("lift_left", m_swerve_model_ptr->getConfig().lift_speed_rpm);
			}
			else if(joy_data->btn_l2){
				rhi_ptr_->setMotorVelocityCommandRPM("lift_right", -m_swerve_model_ptr->getConfig().lift_speed_rpm);
				rhi_ptr_->setMotorVelocityCommandRPM("lift_left", -m_swerve_model_ptr->getConfig().lift_speed_rpm);
			}
			else{
				rhi_ptr_->setMotorVelocityCommandRPM("lift_right", 0);
				rhi_ptr_->setMotorVelocityCommandRPM("lift_left", 0);
			}
		}

		m_digital_io[m_digital_io_name_map.at("claw")] = m_claw_open;


		// Enable Tail Mode
		double tail_mtr_pos = rhi_ptr_->getMotorPosition("tail_motor");
		double stick_turn_offset = m_swerve_model_ptr->getConfig().stick_turn_offset;
		#define MTR_CLOSE_TO(x) (fabs(tail_mtr_pos - x) < stick_turn_offset)

		if(!m_climb_mode && joy_data->btn_l2){
			m_digital_io[m_digital_io_name_map.at("tail")] = true;
			rhi_ptr_->setMotorCurrentLimitMilliAmps("tail_motor", 2500);
			if(joy_data->btn_r2){
				if(!m_tail_mode_btn_pressed){
					// just started pressed
					rhi_ptr_->setMotorPositionCommand("tail_motor", m_swerve_model_ptr->getConfig().stick_angle_max);
					// arbritarily take it to one end, might want to flip this later idk TODO
				}
				m_tail_mode_btn_pressed = true;
				// when beyond extremes, go in opposite direction
				if(MTR_CLOSE_TO(m_swerve_model_ptr->getConfig().stick_angle_min)){
					rhi_ptr_->setMotorPositionCommand("tail_motor", m_swerve_model_ptr->getConfig().stick_angle_max);
				}
				else if(MTR_CLOSE_TO(m_swerve_model_ptr->getConfig().stick_angle_max) ){
					rhi_ptr_->setMotorPositionCommand("tail_motor", m_swerve_model_ptr->getConfig().stick_angle_min);
				} // else stay on course for whatever you're at right now
			}
			else{
				m_tail_mode_btn_pressed = false;
			}
		}
		else{
			rhi_ptr_->setMotorPositionCommand("tail_motor", m_swerve_model_ptr->getConfig().stick_upright_angle);
			if(MTR_CLOSE_TO(m_swerve_model_ptr->getConfig().stick_upright_angle)){ // within n degrees of upright
				m_digital_io[m_digital_io_name_map.at("tail")] = false;
				rhi_ptr_->setMotorCurrentLimitMilliAmps("tail_motor", 300); // i'm going to give it less but not none so it can hold itself centered
			}
		}

		rhi_ptr_->setDigitalIO(m_digital_io);
	}
}

void SwerveRobotPlugin::updateDrivetrainMotors(){
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
		0.0, 0.0, 0.0, 0.0, 0.0, m_curr_odom_cov.z()
	};

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
		sigma_x_vel*sigma_x_vel, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, sigma_y_vel*sigma_y_vel, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, sigma_tht_vel*sigma_tht_vel
	};

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

	RCLCPP_INFO(node_ptr_->get_logger(), "publishing trajectory viz");
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
		double w,x,y,z;
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

	RCLCPP_INFO(node_ptr_->get_logger(), "publishing trajectory arrows");

	m_trajectory_viz_pub->publish(viz_msg);
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)