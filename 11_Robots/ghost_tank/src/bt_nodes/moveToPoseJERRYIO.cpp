/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "ghost_tank/bt_nodes/moveToPoseJERRYIO.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
MoveToPoseJERRYIO::MoveToPoseJERRYIO(const std::string& name, const BT::NodeConfig& config)
: MoveToPose::MoveToPose(name, config){
  	std::cout << "[MoveToPoseJERRYIO::MoveToPoseJERRYIO]" << std::endl;
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPoseJERRYIO::providedPorts(){
	return {
	    BT::InputPort<std::string>("file_path"),
		BT::InputPort<double>("search_radius_tiles"),
		BT::InputPort<double>("xy_exit_threshold_tiles"),
		BT::InputPort<double>("angle_exit_threshold_deg"),
		BT::InputPort<double>("lin_vel_exit_threshold_tps", 1000.0, ""),
		BT::InputPort<double>("ang_vel_exit_threshold_dps", 1000.0, ""),
		BT::InputPort<double>("max_speed_linear_percent"),
		BT::InputPort<double>("max_speed_angular_percent"),
		BT::InputPort<int>("timeout_ms"),
		BT::InputPort<bool>("use_theta"),
		BT::InputPort<bool>("backwards"),
	};
}

void MoveToPoseJERRYIO::GetBlackboardData(){
	file_path = BT_Util::get_input<std::string>(this, "file_path");
	search_radius = BT_Util::get_input<double>(this, "search_radius_tiles", 0.5) * tile_to_meters;
	xy_exit_threshold_m = BT_Util::get_input<double>(this, "xy_exit_threshold_tiles", 0.1) * tile_to_meters;
	angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
	lin_vel_exit_threshold_mps = BT_Util::get_input<double>(this, "lin_vel_exit_threshold_tps", 100.0) * tile_to_meters;
	ang_vel_exit_threshold_radps = BT_Util::get_input<double>(this, "ang_vel_exit_threshold_dps", 1000.0) * ghost_util::DEG_TO_RAD;
	max_speed_linear_percent = BT_Util::get_input<double>(this, "max_speed_linear_percent", 1.0);
	max_speed_angular_percent = BT_Util::get_input<double>(this, "max_speed_angular_percent", 1.0);
	timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
	use_theta = BT_Util::get_input<bool>(this, "use_theta", true);
	backwards = BT_Util::get_input<bool>(this, "backwards", false);
}

void MoveToPoseJERRYIO::FirstLoop(){
	RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseJERRYIO: Started");
	RCLCPP_INFO(node_ptr_->get_logger(), "posX_m: %f", final_pose_.x());
	RCLCPP_INFO(node_ptr_->get_logger(), "posY_m: %f", final_pose_.y());
	RCLCPP_INFO(node_ptr_->get_logger(), "theta_rad: %f", final_pose_.z());
}

void MoveToPoseJERRYIO::GeneratePath(){
	auto path = ghost_util::readPathFromFile(file_path);
	
	std::vector<double> x_trajectory = path[0];
	std::vector<double> y_trajectory = path[1];
	std::vector<double> theta_trajectory = path[2];
	std::vector<double> time_vector;
	
	int num_points = x_trajectory.size();
	for (int i = 0; i <= num_points; i++) {
		// Invert commands when mirrored
		if (BT_Util::get_from_blackboard<bool>(blackboard_, "mirrored")) {
			x_trajectory[i] = 6.0 * tile_to_meters - x_trajectory[i];
			theta_trajectory[i] = ghost_util::WrapAngle2PI(M_PI - theta_trajectory[i]);
		}
		time_vector.push_back(i / static_cast<double>(num_points));
	}
  
	robot_trajectory_.x_trajectory.position_vector = x_trajectory;
	robot_trajectory_.y_trajectory.position_vector = y_trajectory;
	robot_trajectory_.theta_trajectory.position_vector = theta_trajectory;
	robot_trajectory_.x_trajectory.threshold = xy_exit_threshold_m;
	robot_trajectory_.y_trajectory.threshold = xy_exit_threshold_m;
	robot_trajectory_.theta_trajectory.threshold = angle_exit_threshold_rad;
	robot_trajectory_.x_trajectory.time_vector = time_vector;
	robot_trajectory_.y_trajectory.time_vector = time_vector;
	robot_trajectory_.theta_trajectory.time_vector = time_vector;
}

} // namespace ghost_tank
