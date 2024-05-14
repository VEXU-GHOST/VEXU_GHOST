#include "ghost_swerve/trapezoid_motion_planner.hpp"

namespace ghost_swerve {

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using std::placeholders::_1;

void TrapezoidMotionPlanner::initialize(){
	RCLCPP_INFO(node_ptr_->get_logger(), "initializing");

	node_ptr_->declare_parameter("acceleration", "1.0");
	acceleration_ = node_ptr_->get_parameter("acceleration").as_double();

	final_time_ = 1.0;
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
	auto trajectory = std::make_shared<RobotTrajectory>();
	trajectory->x_trajectory = getTrapezoidTraj(acceleration_, v_max, xpos0, xposf);
	trajectory->y_trajectory = getTrapezoidTraj(acceleration_, v_max, xpos0, xposf);
	trajectory->theta_trajectory = getTrapezoidTraj(acceleration_, v_max, xpos0, xposf);

	ghost_msgs::msg::RobotTrajectory trajectory_msg;

	toROSMsg(*trajectory, trajectory_msg);

	trajectory_msg.x_trajectory.threshold = pos_threshold;
	trajectory_msg.y_trajectory.threshold = pos_threshold;
	trajectory_msg.theta_trajectory.threshold = theta_threshold;

	RCLCPP_INFO(node_ptr_->get_logger(), "Generated Swerve Motion Plan");
	trajectory_pub_->publish(trajectory_msg);
}

void TrapezoidMotionPlanner::computeTrapezoidFunction(double accel, double v_max, std::vector<double> vec_q0, std::vector<double> vec_qf){
	bool bad_case = false;
	std::string error = "";
	double pos_initial = vec_q0[0];
	double pos_final = vec_qf[0];
	double vel_initial = vec_q0[1];
	double vel_final = vec_qf[1];

	if(accel <= 0){
		error = "acceleration";
		bad_case = true;
	}
	else if(v_max <= 0){
		error = "v_max";
		bad_case = true;
	}
	else if(pos_final - pos_initial < 0){
		error = "position";
		bad_case = true;
	}
	else if(vel_final > v_max){
		// error = "vel_final";
		// bad_case = true;
		vel_final = v_max;
	}
	if(bad_case){
		badCase(vel_final, error);
	}
	// i think this case is handled below
	// if((vel_final - vel_initial) / accel < 1002131){ //diff in velocity is too high?
	// 	vel_final = v_max;
	// }

	double accel_initial = (v_max - vel_initial > 0.0) ? accel : -accel;
	double accel_final = (vel_final - v_max > 0.0) ? accel : -accel;
	double dist = pos_final - pos_initial;
	double time_accel_initial = abs(v_max - vel_initial) / accel;
	double time_accel_final = abs(v_max - vel_final) / accel;
	double time_1 = time_accel_initial;
	double time_2 = ((v_max - vel_initial) * time_accel_initial / 2 - (vel_final + v_max) * time_accel_final / 2 + dist) / v_max;
	double time_3 = time_2 + time_accel_final;

	if(time_2 < time_1){// doesnt reach max velocity
		if(abs(time_accel_final) <= 0.01){// divide by zero
			computeLinearFunction(dist, vel_initial, vel_final, accel_initial);
		}
		else{
			computeTriangleFunction(dist, vel_initial, vel_final, accel_initial, accel_final);
		}
	}
	else{
		trapezoidVelocityFunc_ = [time_1, time_2, time_3, accel_initial, accel_final, vel_initial, vel_final, v_max](double time){
									 if(time <= time_1){
										 return accel_initial * time + vel_initial;
									 }
									 else if(time <= time_2){
										 return v_max;
									 }
									 else if(time <= time_3){
										 return accel_final * (time - time_2) + v_max;
									 }
									 return vel_final;
								 };
		final_time_ = time_3;
	}
}

void TrapezoidMotionPlanner::computeTriangleFunction(double dist, double vel_initial, double vel_final, double accel_initial, double accel_final){
	double vel_diff = (vel_final - vel_initial);
	double accel_diff = (accel_final - accel_initial);
	double a = accel_diff / 2.0 - accel_diff * accel_diff / (2.0 * accel_final);
	double b = vel_initial * accel_diff / accel_final;
	double c = vel_diff * vel_diff / (2.0 * accel_final) + vel_initial * vel_diff / accel_final - dist;
	double determinant = b * b - 4.0 * a * c;
	if(determinant < 0.0){
		computeLinearFunction(dist, vel_initial, vel_final, accel_initial);
	}
	else{
		double time_1 = (-b + sqrt(determinant)) / (2.0 * a);
		double time_2 = (vel_diff + accel_diff * time_1) / accel_final;

		trapezoidVelocityFunc_ = [time_1, time_2, accel_initial, accel_final, vel_initial, vel_final](double time){
									 if(time <= time_1){
										 return accel_initial * time + vel_initial;
									 }
									 else if(time <= time_2){
										 return accel_final * (time - time_1) + accel_initial * time_1 + vel_final;
									 }
									 return vel_final;
								 };
		final_time_ = time_2;
	}
}

void TrapezoidMotionPlanner::computeLinearFunction(double dist, double vel_initial, double vel_final, double accel_initial){
	double a = accel_initial / 2.0;
	double b = vel_initial;
	double c = -dist;
	double determinant = b * b - 4.0 * a * c;
	if(determinant < 0.0){
		badCase(vel_final, "invalid motion");
	}
	else{
		double time_final = (-b + sqrt(determinant)) / (2.0 * a);
		trapezoidVelocityFunc_ = [time_final, accel_initial, vel_initial, vel_final](double time){
									 if(time <= time_final){
										 return accel_initial * time + vel_initial;
									 }
									 return vel_final;
								 };
		final_time_ = time_final;
	}
}

void TrapezoidMotionPlanner::badCase(double vel_final, std::string error){
	RCLCPP_WARN(node_ptr_->get_logger(), "Warning: Invalid Trajectory Parameters: %s", error.c_str());
	trapezoidVelocityFunc_ = [vel_final](double time){
								 return vel_final;
							 };
}

RobotTrajectory::Trajectory TrapezoidMotionPlanner::getTrapezoidTraj(double accel, double v_max,
                                                                     std::vector<double> vec_q0, 
																	 std::vector<double> vec_qf){
	computeTrapezoidFunction(accel, v_max, vec_q0, vec_qf);

	std::vector<double> position;
	std::vector<double> velocity;
	std::vector<double> time;
	double step = 0.01;
	for(double t = 0; t < final_time_; t += step){
		position.push_back(trapezoidPositionFunc_(t));
		velocity.push_back(trapezoidVelocityFunc_(t));
		time.push_back(t);
	}
	auto trajectory = RobotTrajectory::Trajectory();
	trajectory.position_vector = position;
	trajectory.velocity_vector = velocity;
	trajectory.time_vector = time;
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