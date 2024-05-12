#include "ghost_swerve/trapezoid_motion_planner.hpp"

namespace ghost_swerve {

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using std::placeholders::_1;

void TrapezoidMotionPlanner::initialize(){
	RCLCPP_INFO(node_ptr_->get_logger(), "initializing");
}


void TrapezoidMotionPlanner::generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd){
	RCLCPP_INFO(node_ptr_->get_logger(), "Generating Swerve Motion Plan");

	double theta_f = ghost_util::quaternionToYawRad(cmd->pose.pose.orientation.w,
	                                                cmd->pose.pose.orientation.x,
	                                                cmd->pose.pose.orientation.y,
	                                                cmd->pose.pose.orientation.z);

	// find position/velocities
	std::vector<double> xpos0({current_x_, current_x_vel_});
	std::vector<double> xposf({cmd->pose.pose.position.x, cmd->twist.twist.linear.x});
	std::vector<double> ypos0({current_y_, current_y_vel_});
	std::vector<double> yposf({cmd->pose.pose.position.y, cmd->twist.twist.linear.y});
	std::vector<double> ang0({current_theta_rad_, current_theta_vel_rad_});
	std::vector<double> angf({current_theta_rad_ + ghost_util::SmallestAngleDistRad(theta_f, current_theta_rad_), cmd->twist.twist.angular.z});
	double pos_threshold = cmd->pose.pose.position.z;
	double theta_threshold = cmd->twist.twist.angular.x;

	RCLCPP_INFO(node_ptr_->get_logger(), "current x: %f, current x_vel: %f", xpos0[0], xpos0[1]);
	RCLCPP_INFO(node_ptr_->get_logger(), "current y: %f, current x_vel: %f", ypos0[0], ypos0[1]);
	RCLCPP_INFO(node_ptr_->get_logger(), "current theta: %f, current theta_vel: %f", ang0[0], ang0[1]);
	RCLCPP_INFO(node_ptr_->get_logger(), "final x: %f, final x_vel: %f", xposf[0], xposf[1]);
	RCLCPP_INFO(node_ptr_->get_logger(), "final y: %f, final x_vel: %f", yposf[0], yposf[1]);
	RCLCPP_INFO(node_ptr_->get_logger(), "final theta: %f, final theta_vel: %f", angf[0], angf[1]);

	// find final time
	double v_max = cmd->speed;
	double dist = Eigen::Vector2d(xposf[0] - xpos0[0], yposf[0] - ypos0[0]).norm();
	double t0 = 0;
	double tf = dist / v_max;
	int n = tf * 100;
	auto xpos_traj = getTrapezoidTraj(xpos0, xposf, t0, tf, n);
	auto ypos_traj = getTrapezoidTraj(ypos0, yposf, t0, tf, n);
	auto ang_traj = getTrapezoidTraj(ang0, angf, t0, tf, n);

	ghost_msgs::msg::RobotTrajectory trajectory_msg;
	ghost_msgs::msg::Trajectory x_t;
	ghost_msgs::msg::Trajectory y_t;
	ghost_msgs::msg::Trajectory theta_t;
	// x_t.time = std::get<0>(xpos_traj);
	// x_t.position = std::get<1>(xpos_traj);
	// x_t.velocity = std::get<2>(xpos_traj);
	// y_t.time = std::get<0>(ypos_traj);
	// y_t.position = std::get<1>(ypos_traj);
	// y_t.velocity = std::get<2>(ypos_traj);
	// theta_t.time = std::get<0>(ang_traj);
	// theta_t.position = std::get<1>(ang_traj);
	// theta_t.velocity = std::get<2>(ang_traj);

	x_t.threshold = pos_threshold;
	y_t.threshold = pos_threshold;
	theta_t.threshold = theta_threshold;

	trajectory_msg.x_trajectory = x_t;
	trajectory_msg.y_trajectory = y_t;
	trajectory_msg.theta_trajectory = theta_t;

	RCLCPP_INFO(node_ptr_->get_logger(), "Generated Swerve Motion Plan");
	trajectory_pub_->publish(trajectory_msg);
}

void TrapezoidMotionPlanner::computeTrapezoidFunction(double time, double accel, double v_max, std::vector<double> vec_q0, std::vector<double> vec_qf){
	// vec_q0 = {position, velocity}
	// Ax = B
	// A = trapezoid function matrix
	// x = coefficients
	// B = initial and final values
	if(accel <= 0){
		throw"you suck";
	}
	if(time < 0){
		throw"you suck";
	}
	if(v_max <= 0){
		throw"you suck";
	}

	double pos_initial = vec_q0[0];
	double pos_final = vec_qf[0];
	double vel_initial = vec_q0[1];
	double vel_final = vec_qf[1];

	if(pos_final - pos_initial < 0){
		throw"you suck";
	}
	if((vel_final - vel_initial) / accel < 1002131){
		vel_final = v_max;
	}

	double accel_start = (v_max - vel_initial > 0.0) ? accel : -accel;
	double accel_end = (vel_final - v_max > 0.0) ? accel : -accel;
	double dist = pos_final - pos_initial;
	double time_accel_initial = abs(v_max - vel_initial) / accel;
	double time_accel_final = abs(v_max - vel_final) / accel;
	double time_1 = time_accel_initial;
	double time_2 = ((v_max - vel_initial) * time_accel_initial / 2 - (vel_final + v_max) * time_accel_final / 2 + dist) / v_max;
	double time_3 = time_2 + time_accel_final;

	if(time_2 - time_1 < 0){// doesnt reach max velocity
		if(abs(time_accel_final) <= 0.01){// divide by zero
			throw"use solver 3";
		}
		throw"use solver 2";
	}

	trapezoidVelocityFunc_ = [time, time_1, time_2, time_3, accel_start, accel_end, vel_initial, vel_final, v_max]{							 
		if(time <= time_1){
			return accel_start * time + vel_initial;
		}
		else if(time <= time_2){
			return v_max;
		}
		else if(time <= time_3){
			return accel_end * (time - time_2) + v_max;
		}
		return vel_final;
	};

}

RobotTrajectory::Trajectory TrapezoidMotionPlanner::getTrapezoidTraj(std::vector<double> vec_q0,
                                                                     std::vector<double> vec_qf,
                                                                     double t0, double tf, int n){
	// A = coefficients
	// n = number of timesteps
	// auto A = computeTrapezoidCoeff(t0, tf, vec_q0, vec_qf);

	// std::vector<double> a = {A(0, 0), A(1, 0), A(2, 0), A(3, 0)};

	std::vector<double> qd;
	std::vector<double> d_qd;
	std::vector<double> dd_qd;
	std::vector<double> time;
	double step = (tf - t0) / n;
	for(double t = t0; t < tf; t += step){
		// double qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3);
		// double d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2);

		// qd.push_back(qdi);
		// d_qd.push_back(d_qdi);
		time.push_back(t);
	}
	auto trajectory = RobotTrajectory::Trajectory();
	// return std::make_tuple(time, qd, d_qd);
	return trajectory;
}

} // namespace ghost_swerve

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);

	auto trapezoid_motion_planner = std::make_shared<ghost_swerve::TrapezoidMotionPlanner>();
	trapezoid_motion_planner->configure("trapezoid_motion_planner");
	auto trapezoid_motion_planner_node = trapezoid_motion_planner->getROSNodePtr();
	rclcpp::spin(trapezoid_motion_planner_node);
	rclcpp::shutdown();
	return 0;
}