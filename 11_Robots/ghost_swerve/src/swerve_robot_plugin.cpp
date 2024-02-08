#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iostream>

using std::placeholders::_1;
using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;

namespace ghost_swerve {

SwerveRobotPlugin::SwerveRobotPlugin(){
}

void SwerveRobotPlugin::initialize(){
	std::cout << "Swerve Robot Initialization!" << std::endl;

	node_ptr_->declare_parameter("trajectory_topic", "/motion_planning/trajectory");
	std::string trajectory_topic = node_ptr_->get_parameter("trajectory_topic").as_string();

	node_ptr_->declare_parameter("odom_topic", "/sensors/odom");
	std::string odom_topic = node_ptr_->get_parameter("odom_topic").as_string();

	node_ptr_->declare_parameter<std::string>("bt_path");
	bt_path_ = node_ptr_->get_parameter("bt_path").as_string();

	trajectory_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::RobotTrajectory>(
		trajectory_topic,
		10,
		std::bind(&SwerveRobotPlugin::trajectoryCallback, this, _1)
		);

	// odom_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
	// 	odom_topic,
	// 	10);

	bt_ = std::make_shared<RunTree>(bt_path_, robot_hardware_interface_ptr_);
}
void SwerveRobotPlugin::disabled(){
}
void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;

	bt_->tick_tree();

	update_motor_commands(current_time - trajectory_start_time_);

	// update motor values from trajectory
}
void SwerveRobotPlugin::teleop(double current_time){
	// std::cout << "Teleop: " << current_time << std::endl;

	// for testing
	autonomous(current_time);

	// auto joy_data = robot_hardware_interface_ptr_->getMainJoystickData();

	if(joy_data->btn_a){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_frr", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_frr", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_b){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_frl", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_frl", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_x){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_fll", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_fll", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_y){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_flr", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_flr", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_u){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_brb", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_brb", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_l){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_brf", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_brf", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_r){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_blf", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_blf", joy_data->right_y / 127.0);
	}
	if(joy_data->btn_d){
		robot_hardware_interface_ptr_->setMotorCurrentLimitMilliAmps("drive_blb", 2500);
		robot_hardware_interface_ptr_->setMotorVoltageCommandPercent("drive_blb", joy_data->right_y / 127.0);
	}
}

// does this need to be a part of the base class?
void SwerveRobotPlugin::trajectoryCallback(const ghost_msgs::msg::RobotTrajectory::SharedPtr msg){
	trajectory_start_time_ = getTimeFromStart();
	for (int i = 0; i < msg->motor_names.size(); i++){
		RobotTrajectory::MotorTrajectory motor_trajectory;
		fromROSMsg(motor_trajectory, msg->trajectories[i]);
		trajectory_motor_map_[msg->motor_names[i]] = motor_trajectory;
	}
}

void SwerveRobotPlugin::update_motor_commands(double time){
	for (auto& [motor_name, motor_trajectory] : trajectory_motor_map_){
		const auto [is_pos_command, position] = motor_trajectory.getPosition(time);
		if (is_pos_command){
			robot_hardware_interface_ptr_->setMotorPositionCommand(motor_name, position);
		}
		const auto [is_torque_command, torque] = motor_trajectory.getTorque(time);
		if (is_torque_command){
			robot_hardware_interface_ptr_->setMotorTorqueCommandPercent(motor_name, torque);
		}
		const auto [is_velocity_command, velocity] = motor_trajectory.getVelocity(time);
		if (is_velocity_command){
			robot_hardware_interface_ptr_->setMotorVelocityCommandRPM(motor_name, velocity);
		}
		const auto [is_voltage_command, voltage] = motor_trajectory.getVoltage(time);
		if (is_voltage_command){
			robot_hardware_interface_ptr_->setMotorVoltageCommandPercent(motor_name, voltage);
		}
	}
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)