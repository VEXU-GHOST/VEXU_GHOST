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

#include "alpha_jerry/bt_nodes/moveToPosePurepursuit.hpp"


using std::placeholders::_1;

namespace alpha_jerry
{

// If your Node has ports, you must use this constructor signature
MoveToPosePurepursuit::MoveToPosePurepursuit(const std::string& name, const BT::NodeConfig& config):
	BT::StatefulActionNode(name, config){
  	std::cout << "[MoveToPosePurepursuit::MoveToPosePurepursuit]" << std::endl;
		
	blackboard_ = config.blackboard;
	BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
	BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);

	if(!node_ptr_->has_parameter("behavior_tree.trajectory_topic")){
		node_ptr_->declare_parameter(
		"behavior_tree.trajectory_topic",
		"/motion_planner/trajectory");
	}
	std::string Purepursuit_planner_topic =
		node_ptr_->get_parameter("behavior_tree.trajectory_topic").as_string();

	trajectory_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::RobotTrajectory>(
		Purepursuit_planner_topic,
		10);

	started_ = false;

	
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPosePurepursuit::providedPorts(){
	return {
	    BT::InputPort<std::string>("fileName"),
		BT::InputPort<double>("threshold"),
		BT::InputPort<double>("angle_threshold"),
		BT::InputPort<int>("timeout"),
		BT::InputPort<bool>("use_theta"),
	};
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPosePurepursuit::onStart(){
	started_ = false;
	// plan_time_ = std::chrono::();
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPosePurepursuit::onHalted(){
	resetStatus();
}

BT::NodeStatus MoveToPosePurepursuit::onRunning() {
	std::string file_name = BT_Util::get_input<std::string>(this, "fileName");
	double threshold = BT_Util::get_input<double>(this, "threshold");
	double angle_threshold = BT_Util::get_input<double>(this, "angle_threshold");
	int timeout = BT_Util::get_input<int>(this, "timeout");
	bool use_theta = BT_Util::get_input<bool>(this, "use_theta");

	std::vector<double> x_values; 
	std::vector<double> y_values; 
	std::vector<double> angle_values; 

	ghost_util::readPathFromFile(file_name, x_values, y_values, angle_values);
	int length = x_values.size();

	ghost_msgs::msg::RobotTrajectory msg{};
	msg.header.stamp = node_ptr_->get_clock()->now();
	msg.x_trajectory.position = x_values;
	msg.y_trajectory.position = y_values;
	msg.theta_trajectory.position = angle_values;
	msg.x_trajectory.threshold = threshold;
	msg.y_trajectory.threshold = threshold;
	msg.theta_trajectory.threshold = angle_threshold;
	
	msg.x_trajectory.time = {0.0};
	msg.y_trajectory.time = {0.0};
	msg.theta_trajectory.time = {0.0};

	msg.trajectory_type = ghost_msgs::msg::RobotTrajectory::TRAJECTORY_TYPE_PUREPURSUIT;

	if (use_theta){
		if( (abs(x_values[length-1] - tank_model_ptr_->getWorldPose().x()) < threshold) &&
			(abs(y_values[length-1] - tank_model_ptr_->getWorldPose().y()) < threshold)
			 && (abs(ghost_util::SmallestAngleDistRad(angle_values[length-1], tank_model_ptr_->getWorldAngleRad())) < angle_threshold)
			)
		{
			RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPosePurepursuit: Success");
			return BT::NodeStatus::SUCCESS;
		}
	} else {
		if( (abs(x_values[length-1] - tank_model_ptr_->getWorldPose().x()) < threshold) &&
			(abs(y_values[length-1] - tank_model_ptr_->getWorldPose().y()) < threshold) 
			)
		{
			RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPosePurepursuit: Success");
			return BT::NodeStatus::SUCCESS;
		}
	}

	if(started_){
		auto now = std::chrono::system_clock::now();
		int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
		// int time_elapsed_since_plan = std::chrono::duration_cast<std::chrono::milliseconds>(now - plan_time_).count();
		// RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPosePurepursuit: %i ms elapsed", time_elapsed);
		if(timeout > 0){ // positive timeout means how often to plan/send trajectory
			if(time_elapsed > timeout){
				RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPosePurepursuit Timeout: %i ms elapsed", time_elapsed);
				// started_ = false;
				// return BT::NodeStatus::FAILURE;
			// } else if (time_elapsed_since_plan > 10000){
				start_time_ = std::chrono::system_clock::now();
				trajectory_pub_->publish(msg);
				RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPosePurepursuit: sent trajectory");
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
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPosePurepursuit: Started");
		start_time_ = std::chrono::system_clock::now();
		started_ = true;
		trajectory_pub_->publish(msg);

	
  	}

  	return BT::NodeStatus::RUNNING;
}

} // namespace alpha_jerry
