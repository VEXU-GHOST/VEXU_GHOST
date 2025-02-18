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

#include "ghost_tank/bt_nodes/moveToPoseBoomerang.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
MoveToPoseBoomerang::MoveToPoseBoomerang(const std::string& name, const BT::NodeConfig& config):
	BT::StatefulActionNode(name, config){
  	std::cout << "[MoveToPoseBoomerang::MoveToPoseBoomerang]" << std::endl;
		
	blackboard_ = config.blackboard;
	BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
	BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);

	if(!node_ptr_->has_parameter("behavior_tree.trajectory_topic")){
		node_ptr_->declare_parameter(
		"behavior_tree.trajectory_topic",
		"/motion_planner/trajectory");
	}
	std::string boomerang_planner_topic =
		node_ptr_->get_parameter("behavior_tree.trajectory_topic").as_string();

	trajectory_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::RobotTrajectory>(
		boomerang_planner_topic,
		10);

	started_ = false;

	boomerang_ = std::make_shared<Boomerang>();
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPoseBoomerang::providedPorts(){
	return {
	    BT::InputPort<double>("posX"),
	    BT::InputPort<double>("posY"),
	    BT::InputPort<double>("theta"),
	    BT::InputPort<double>("threshold"),
	    BT::InputPort<double>("angle_threshold"),
	    BT::InputPort<double>("lead"),
	    BT::InputPort<int>("timeout"),
		BT::InputPort<bool>("use_theta"),
	};
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPoseBoomerang::onStart(){
	started_ = false;
	// plan_time_ = std::chrono::();
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPoseBoomerang::onHalted(){
	resetStatus();
}

BT::NodeStatus MoveToPoseBoomerang::onRunning() {
	double posX = BT_Util::get_input<double>(this, "posX");
	double posY = BT_Util::get_input<double>(this, "posY");
	double theta = BT_Util::get_input<double>(this, "theta");
	double threshold = BT_Util::get_input<double>(this, "threshold");
	double angle_threshold = BT_Util::get_input<double>(this, "angle_threshold");
	double lead = BT_Util::get_input<double>(this, "lead");
	int timeout = BT_Util::get_input<int>(this, "timeout");
	bool use_theta = BT_Util::get_input<bool>(this, "use_theta");
	double tile_to_meters = 0.6096;
	posX *= tile_to_meters;
	posY *= tile_to_meters;

	theta *= ghost_util::DEG_TO_RAD;
	angle_threshold *= ghost_util::DEG_TO_RAD;

	double w, x, y, z;
	ghost_util::yawToQuaternionRad(theta, w, x, y, z);

	boomerang_->set_end_point(posX, posY, theta);
	boomerang_->set_lead(lead);
	boomerang_->map_curve(tank_model_ptr_->getWorldPose());
	auto points = boomerang_->get_points();
	std::vector<double> x_trajectory;
    std::vector<double> y_trajectory;
    std::vector<double> theta_trajectory;
	std::vector<double> time_vector;

    for (const auto& vec : points) {
        x_trajectory.push_back(vec.x());
        y_trajectory.push_back(vec.y());
		theta_trajectory.push_back(theta);
    }
	for (int i = 0; i <= 50; i++){
		time_vector.push_back(i * 1.0/50.0);
	}

	ghost_msgs::msg::RobotTrajectory msg{};
	msg.header.stamp = node_ptr_->get_clock()->now();
	msg.x_trajectory.position = x_trajectory;
	msg.y_trajectory.position = y_trajectory;
	msg.theta_trajectory.threshold = angle_threshold;
	msg.x_trajectory.threshold = threshold;
	msg.y_trajectory.threshold = threshold;
	msg.theta_trajectory.position = theta_trajectory;

	msg.x_trajectory.time = time_vector;
	msg.y_trajectory.time = time_vector;
	msg.theta_trajectory.time = time_vector;

	msg.trajectory_type = ghost_msgs::msg::RobotTrajectory::TRAJECTORY_TYPE_PUREPURSUIT;

	double dist_err = sqrt((posX - tank_model_ptr_->getWorldPose().x()) * (posX - tank_model_ptr_->getWorldPose().x()) + 
	(posY - tank_model_ptr_->getWorldPose().y()) * (posY - tank_model_ptr_->getWorldPose().y()));

	if (use_theta){
		if(dist_err < threshold && (abs(ghost_util::SmallestAngleDistRad(theta, tank_model_ptr_->getWorldAngleRad())) < angle_threshold))
		{
			RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: Success");
			return BT::NodeStatus::SUCCESS;
		}
	} else {
		if(dist_err < threshold)
		{
			RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: Success");
			return BT::NodeStatus::SUCCESS;
		}
	}


	if(started_){
		auto now = std::chrono::system_clock::now();
		int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
		// int time_elapsed_since_plan = std::chrono::duration_cast<std::chrono::milliseconds>(now - plan_time_).count();
		// RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: %i ms elapsed", time_elapsed);
		if(timeout >= 0){ // positive timeout means how often to plan/send trajectory
			if(time_elapsed > timeout){
				RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPoseBoomerang Timeout: %i ms elapsed", time_elapsed);
				// started_ = false;
				// return BT::NodeStatus::FAILURE;
			// } else if (time_elapsed_since_plan > 10000){
				start_time_ = std::chrono::system_clock::now();
				trajectory_pub_->publish(msg);
				RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: sent trajectory");
				// plan_time_ = std::chrono::system_clock::now();
			}
		}
		else{ // negative timeout means how long to wait until move on to the next command
			if(time_elapsed > abs(timeout)){
				return BT::NodeStatus::SUCCESS;
			}
		}
	}
	else{
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: Started");
		start_time_ = std::chrono::system_clock::now();
		started_ = true;
		trajectory_pub_->publish(msg);

		RCLCPP_INFO(node_ptr_->get_logger(), "posX: %f", posX);
		RCLCPP_INFO(node_ptr_->get_logger(), "posY: %f", posY);
		RCLCPP_INFO(node_ptr_->get_logger(), "theta: %f", theta);
  	}

  	return BT::NodeStatus::RUNNING;
}

} // namespace ghost_tank
