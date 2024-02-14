#include "ghost_swerve/swerve_motion_planner.hpp"

namespace ghost_swerve {

using ghost_planners::CubicMotionPlanner;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using std::placeholders::_1;

void SwerveMotionPlanner::initialize(){
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
		"/odometry/filtered",
		10,
		std::bind(&SwerveMotionPlanner::odomCallback, this, _1)
		);

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

void SwerveMotionPlanner::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg){
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;
	current_x_vel = msg->twist.twist.linear.x;
	current_y_vel = msg->twist.twist.linear.x;

	tf2::Quaternion quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	current_angle = quat.getAngle();
	current_omega = msg->twist.twist.angular.x;
}


void SwerveMotionPlanner::generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd){
	RCLCPP_INFO(get_logger(), "Generating Swerve Motion Plan");

	tf2::Quaternion orientation_f(cmd->pose.pose.orientation.x,
	                              cmd->pose.pose.orientation.y,
	                              cmd->pose.pose.orientation.z,
	                              cmd->pose.pose.orientation.w);

	// put into cubic for x,y,angle as joystick left_x, left_y, right_x

	// find position/velocities
	std::vector<double> xpos0({current_x, current_x_vel});
	std::vector<double> xposf({cmd->pose.pose.position.x, cmd->twist.twist.linear.x});
	std::vector<double> ypos0({current_x, current_x_vel});
	std::vector<double> yposf({cmd->pose.pose.position.x, cmd->twist.twist.linear.x});
	std::vector<double> ang0({0, current_omega});
	std::vector<double> angf({ghost_util::SmallestAngleDistRad(current_angle, orientation_f.getAngle()), cmd->twist.twist.angular.x});

	// find final time
	double v_max = 0.5;
	// double a_max = 0.5;
	double dist = Eigen::Vector2d(posf[0], posf[1]).norm();
	double t0 = 0;
	double tf = dist / v_max;
	int n = tf*100;
	auto xpos_traj = CubicMotionPlanner::computeCubicTraj(xpos0, xposf, t0, tf, n);
	auto ypos_traj = CubicMotionPlanner::computeCubicTraj(xpos0, xposf, t0, tf, n);
	auto ang_traj = CubicMotionPlanner::computeCubicTraj(ang0, angf, t0, tf, n);
	// auto time = std::get<0>(xpos_traj);
	// auto xpos_vel = std::get<2>(xpos_traj);
	// auto ypos_vel = std::get<2>(ypos_traj);
	// auto ang_vel = std::get<2>(ang_traj);

	// if not field control
	// std::vector<double> x_velocity;
	// std::vector<double> y_velocity;
	// for(double v : pos_vel){
	// 	x_velocity.push_back(v * cos(current_angle));
	// 	y_velocity.push_back(v * sin(current_angle));
	// }

	// auto swerve_trajectory = motor_commands_from_joystick(x_velocity, y_velocity, ang_vel);
	// ghost_msgs::msg::RobotTrajectory trajectory_msg;
	// toROSMsg(swerve_trajectory, trajectory_msg);

	ghost_msgs::msg::RobotTrajectory trajectory_msg;
	ghost_msgs::msg::MotorTrajectory x_mt;
	ghost_msgs::msg::MotorTrajectory y_mt;
	ghost_msgs::msg::MotorTrajectory angle_mt;
	x_mt.time = std::get<0>(xpos_traj);
	x_mt.position = std::get<1>(xpos_traj);
	x_mt.velocity = std::get<2>(xpos_traj);
	y_mt.time = std::get<0>(ypos_traj);
	y_mt.position = std::get<1>(ypos_traj);
	y_mt.velocity = std::get<2>(ypos_traj);
	angle_mt.time = std::get<0>(ang_traj);
	angle_mt.position = std::get<1>(ang_traj);
	angle_mt.velocity = std::get<2>(ang_traj);
	trajectory_msg.motor_names = std::vector<std::string>({"x","y","angle"});

	trajectory_msg.trajectories = std::vector<ghost_msgs::msg::MotorTrajectoy>({x_mt, y_mt, angle_mt});
	trajectory_pub_->publish(trajectory_msg);
	// boom
}

ghost_planners::RobotTrajectory SwerveMotionPlanner::motor_commands_from_joystick(std::vector<double> x_vel,std::vector<double> y_vel,std::vector<double> omega){
	ghost_planners::RobotTrajectory robot_trajectory;
	robot_trajectory.motor_names = {"drive_flr", "drive_fll", "drive_frr", "drive_frl",
		                            "drive_blf", "drive_blb", "drive_brf", "drive_brb"};
	ghost_planners::RobotTrajectory::MotorTrajectory flr_trajectory, fll_trajectory, frr_trajectory, frl_trajectory,
	                                                 blf_trajectory, blb_trajectory, brf_trajectory, brb_trajectory;
	RCLCPP_INFO(get_logger(), "motor trajectory initialized");

	std::unordered_map<std::string, std::pair<std::string, std::string> > module_actuator_motor_mapping{
		{"left_front", std::pair<std::string, std::string>("drive_flr", "drive_fll")},
		{"right_front", std::pair<std::string, std::string>("drive_frr", "drive_frl")},
		{"left_back", std::pair<std::string, std::string>("drive_blf", "drive_blb")},
		{"right_back", std::pair<std::string, std::string>("drive_brf", "drive_brb")}
	};

	std::unordered_map<std::string, std::shared_ptr<ghost_planners::RobotTrajectory::MotorTrajectory> > trajectory_motor_mapping{
		{"drive_flr", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(flr_trajectory)},
		{"drive_fll", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(fll_trajectory)},
		{"drive_frr", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(frr_trajectory)},
		{"drive_frl", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(frl_trajectory)},
		{"drive_blf", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(blf_trajectory)},
		{"drive_blb", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(blb_trajectory)},
		{"drive_brf", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(brf_trajectory)},
		{"drive_brb", std::make_shared<ghost_planners::RobotTrajectory::MotorTrajectory>(brb_trajectory)}
	};
	RCLCPP_INFO(get_logger(), "maps initialized");

	for(int i = 0; i < x_vel.size(); i++){
		m_swerve_model_ptr->calculateKinematicSwerveController(x_vel[i], y_vel[i], omega[i]);

		for(const auto & [module_name, motor_name_pair] : module_actuator_motor_mapping){
			std::string m1_name = motor_name_pair.first;
			std::string m2_name = motor_name_pair.second;
			auto command = m_swerve_model_ptr->getModuleCommand(module_name);

			trajectory_motor_mapping[m1_name]->velocity_vector.push_back(command.actuator_velocity_commands[0]);
			trajectory_motor_mapping[m1_name]->voltage_vector.push_back(command.actuator_voltage_commands[0]);

			trajectory_motor_mapping[m2_name]->velocity_vector.push_back(command.actuator_velocity_commands[1]);
			trajectory_motor_mapping[m2_name]->voltage_vector.push_back(command.actuator_voltage_commands[1]);
		}
	}

	RCLCPP_INFO(get_logger(), "motor trajectory done");

	robot_trajectory.motor_trajectories = {flr_trajectory, fll_trajectory, frr_trajectory, frl_trajectory,
		                                   blf_trajectory, blb_trajectory, brf_trajectory, brb_trajectory};
	return robot_trajectory;
}

} // namespace ghost_swerve

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);

	auto swerve_motion_planner_node = std::make_shared<ghost_swerve::SwerveMotionPlanner>();
	swerve_motion_planner_node->configure();
	rclcpp::spin(swerve_motion_planner_node);
	rclcpp::shutdown();
	return 0;
}