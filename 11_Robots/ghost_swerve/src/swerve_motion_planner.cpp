#include "ghost_swerve/swerve_motion_planner.hpp"

namespace ghost_swerve {

using ghost_planners::CubicMotionPlanner;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using std::placeholders::_1;

void SwerveMotionPlanner::initialize(){
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
		"/odom_ekf/odometry",
		10,
		std::bind(&SwerveMotionPlanner::odomCallback, this, _1)
		);
}

void SwerveMotionPlanner::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg){
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;
	current_x_vel = msg->twist.twist.linear.x;
	current_y_vel = msg->twist.twist.linear.y;

	current_angle = ghost_util::quaternionToYawRad(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	current_omega = msg->twist.twist.angular.z;
}


void SwerveMotionPlanner::generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd){
	RCLCPP_INFO(get_logger(), "Generating Swerve Motion Plan");

	double angle_f = ghost_util::quaternionToYawRad(cmd->pose.pose.orientation.w,
	                                                cmd->pose.pose.orientation.x,
	                                                cmd->pose.pose.orientation.y,
	                                                cmd->pose.pose.orientation.z);

	// put into cubic for x,y,angle as joystick left_x, left_y, right_x

	// find position/velocities
	std::vector<double> xpos0({current_x, current_x_vel});
	std::vector<double> xposf({cmd->pose.pose.position.x, cmd->twist.twist.linear.x});
	std::vector<double> ypos0({current_y, current_y_vel});
	std::vector<double> yposf({cmd->pose.pose.position.y, cmd->twist.twist.linear.y});
	std::vector<double> ang0({current_angle, current_omega});
	std::vector<double> angf({current_angle + ghost_util::SmallestAngleDistRad(angle_f, current_angle), cmd->twist.twist.angular.z});
	double threshold = cmd->pose.pose.position.z;
	double angle_threshold = cmd->twist.twist.angular.x;

	RCLCPP_INFO(get_logger(), "current x: %f, current x_vel: %f", xpos0[0], xpos0[1]);
	RCLCPP_INFO(get_logger(), "current y: %f, current x_vel: %f", ypos0[0], ypos0[1]);
	RCLCPP_INFO(get_logger(), "current theta: %f, current theta_vel: %f", ang0[0], ang0[1]);
	RCLCPP_INFO(get_logger(), "final x: %f, final x_vel: %f", xposf[0], xposf[1]);
	RCLCPP_INFO(get_logger(), "final y: %f, final x_vel: %f", yposf[0], yposf[1]);
	RCLCPP_INFO(get_logger(), "final theta: %f, final theta_vel: %f", angf[0], angf[1]);


	// find final time
	double v_max = 0.5;
	// double a_max = 0.5;
	double dist = Eigen::Vector2d(xposf[0] - xpos0[0], yposf[0] - ypos0[0]).norm();
	double t0 = 0;
	double tf = 1.5 * dist / v_max;
	int n = tf * 100;
	auto xpos_traj = CubicMotionPlanner::computeCubicTraj(xpos0, xposf, t0, tf, n);
	auto ypos_traj = CubicMotionPlanner::computeCubicTraj(ypos0, yposf, t0, tf, n);
	auto ang_traj = CubicMotionPlanner::computeCubicTraj(ang0, angf, t0, tf, n);

	ghost_msgs::msg::RobotTrajectory trajectory_msg;
	ghost_msgs::msg::MotorTrajectory x_mt;
	ghost_msgs::msg::MotorTrajectory y_mt;
	ghost_msgs::msg::MotorTrajectory angle_mt;
	ghost_msgs::msg::MotorTrajectory threshold_mt;
	x_mt.time = std::get<0>(xpos_traj);
	x_mt.position = std::get<1>(xpos_traj);
	x_mt.velocity = std::get<2>(xpos_traj);
	y_mt.time = std::get<0>(ypos_traj);
	y_mt.position = std::get<1>(ypos_traj);
	y_mt.velocity = std::get<2>(ypos_traj);
	angle_mt.time = std::get<0>(ang_traj);
	angle_mt.position = std::get<1>(ang_traj);// + current_angle;
	angle_mt.velocity = std::get<2>(ang_traj);

	threshold_mt.time = {0};
	threshold_mt.position = {threshold};
	threshold_mt.velocity = {angle_threshold};

	trajectory_msg.motor_names = std::vector<std::string>({"x","y","angle","threshold"});

	trajectory_msg.trajectories = std::vector<ghost_msgs::msg::MotorTrajectory>({x_mt, y_mt, angle_mt, threshold_mt});
	RCLCPP_INFO(get_logger(), "Generated Swerve Motion Plan");
	trajectory_pub_->publish(trajectory_msg);
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