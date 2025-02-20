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
#include "ghost_tank/pdcontrol.hpp"

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
	BT_Util::get_from_blackboard(blackboard_, "pd_control_ptr", pd_control_ptr_);

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
	    BT::InputPort<double>("posX_tiles"),
	    BT::InputPort<double>("posY_tiles"),
	    BT::InputPort<double>("theta_deg"),
		BT::InputPort<double>("search_radius_m"),
	    BT::InputPort<double>("threshold_m"),
	    BT::InputPort<double>("angle_threshold_deg"),
		BT::InputPort<double>("threshold_vel_mps"),
	    BT::InputPort<double>("angle_threshold_vel_dps"),
	    BT::InputPort<double>("lead"),
		BT::InputPort<double>("max_speed_linear_pct"),
		BT::InputPort<double>("max_speed_angular_pct"),
	    BT::InputPort<int>("timeout_ms"),
		BT::InputPort<bool>("use_theta"),
		BT::InputPort<bool>("backwards"),
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
	bool mirrored = false;
	BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);

	double posX = BT_Util::get_input<double>(this, "posX_tiles");
	double posY = BT_Util::get_input<double>(this, "posY_tiles");
	double theta = BT_Util::get_input<double>(this, "theta_deg");
	double threshold = BT_Util::get_input<double>(this, "threshold_m");
	double angle_threshold = BT_Util::get_input<double>(this, "angle_threshold_deg");
	double threshold_vel = BT_Util::get_input<double>(this, "threshold_vel_mps");
	double angle_threshold_vel = BT_Util::get_input<double>(this, "angle_threshold_vel_dps");
	int timeout = BT_Util::get_input<int>(this, "timeout_ms");
	bool use_theta = BT_Util::get_input<bool>(this, "use_theta");
	bool backwards = BT_Util::get_input<bool>(this, "backwards");

	if (mirrored){
		posX = 6.0 - posX;
		theta = 180.0 - theta;
	}

	double tile_to_meters = 0.6096;
	posX *= tile_to_meters;
	posY *= tile_to_meters;
	
	theta *= ghost_util::DEG_TO_RAD;
	angle_threshold *= ghost_util::DEG_TO_RAD;
	

	if(backwards){
		theta = ghost_util::FlipAnglePI(theta);
	}

	double dist_err = sqrt((posX - tank_model_ptr_->getWorldPose().x()) * (posX - tank_model_ptr_->getWorldPose().x()) + 
	(posY - tank_model_ptr_->getWorldPose().y()) * (posY - tank_model_ptr_->getWorldPose().y()));

	// std::cout << "dist_err: " << dist_err << std::endl;
	// std::cout << "ang_err:  " << ghost_util::SmallestAngleDistRad(theta, tank_model_ptr_->getWorldAngleRad())*ghost_util::RAD_TO_DEG << std::endl;

	if (use_theta){
		if(dist_err < threshold && (abs(ghost_util::SmallestAngleDistRad(theta, tank_model_ptr_->getWorldAngleRad())) < angle_threshold)
		&& (abs(tank_model_ptr_->getWorldTwist().x()) < threshold_vel) && (abs(tank_model_ptr_->getWorldTwist().z())*ghost_util::RAD_TO_DEG < angle_threshold_vel))
		{
			RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: Success");
			return BT::NodeStatus::SUCCESS;
		}
	} else {
		if(dist_err < threshold
		&& (abs(tank_model_ptr_->getWorldTwist().x()) < threshold_vel) && (abs(tank_model_ptr_->getWorldTwist().z())*ghost_util::RAD_TO_DEG < angle_threshold_vel))
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
				start_time_ = std::chrono::system_clock::now();
				GeneratePath();
				RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: Replanned");
			}
		}
		else{ // negative timeout means how long to wait until move on to the next command
			if(time_elapsed > abs(timeout)){
				RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPoseBoomerang: Skipped");
				return BT::NodeStatus::SUCCESS;
			}
		}
	}
	else{
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseBoomerang: Started");
		start_time_ = std::chrono::system_clock::now();
		started_ = true;
		GeneratePath();

		RCLCPP_INFO(node_ptr_->get_logger(), "posX: %f", posX);
		RCLCPP_INFO(node_ptr_->get_logger(), "posY: %f", posY);
		RCLCPP_INFO(node_ptr_->get_logger(), "theta: %f", theta);
  	}

	PurePursuit();
	publishTrajectoryVisualization();

  	return BT::NodeStatus::RUNNING;
}

void MoveToPoseBoomerang::GeneratePath(){
	double posX = BT_Util::get_input<double>(this, "posX_tiles");
	double posY = BT_Util::get_input<double>(this, "posY_tiles");
	double theta = BT_Util::get_input<double>(this, "theta_deg");
	double threshold_xy = BT_Util::get_input<double>(this, "threshold_m");
	double threshold_theta = BT_Util::get_input<double>(this, "angle_threshold_deg");
	double lead = BT_Util::get_input<double>(this, "lead");

	double tile_to_meters = 0.6096;
	posX *= tile_to_meters;
	posY *= tile_to_meters;

	theta *= ghost_util::DEG_TO_RAD;
	threshold_theta *= ghost_util::DEG_TO_RAD;

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

	robot_trajectory_.x_trajectory.position_vector = x_trajectory;
	robot_trajectory_.y_trajectory.position_vector = y_trajectory;
	robot_trajectory_.theta_trajectory.position_vector = theta_trajectory;
	robot_trajectory_.x_trajectory.threshold = threshold_xy;
	robot_trajectory_.y_trajectory.threshold = threshold_xy;
	robot_trajectory_.theta_trajectory.threshold = threshold_theta;
	robot_trajectory_.x_trajectory.time_vector = time_vector;
	robot_trajectory_.y_trajectory.time_vector = time_vector;
	robot_trajectory_.theta_trajectory.time_vector = time_vector;
}

void MoveToPoseBoomerang::PurePursuit(){
	double search_radius = BT_Util::get_input<double>(this, "search_radius_m");
	double max_speed_linear = BT_Util::get_input<double>(this, "max_speed_linear_pct");
	double max_speed_angular = BT_Util::get_input<double>(this, "max_speed_angular_pct");
	bool backwards = BT_Util::get_input<bool>(this, "backwards");

	double current_x = tank_model_ptr_->getWorldPose().x();
	double current_y = tank_model_ptr_->getWorldPose().y();
	double current_angle = tank_model_ptr_->getWorldAngleRad();

	auto x_trajectory = robot_trajectory_.x_trajectory.position_vector;
	auto y_trajectory = robot_trajectory_.y_trajectory.position_vector;
	auto theta_trajectory = robot_trajectory_.theta_trajectory.position_vector;

	auto threshold_xy = robot_trajectory_.x_trajectory.threshold;
	auto threshold_theta = robot_trajectory_.theta_trajectory.threshold;

	Eigen::Vector3d desired_pose;
	Eigen::Vector3d final_pose;

	past_index_ = 0;

    for (int i = past_index_; i < x_trajectory.size(); ++i) {//find farthest point in radius
      double distance = sqrt(
        pow((current_x - x_trajectory[i]), 2) +
        pow((current_y - y_trajectory[i]), 2));
      if (distance < search_radius) {
        past_index_ = i;
      }
    }

    desired_pose = Eigen::Vector3d(x_trajectory[past_index_], y_trajectory[past_index_], 0.0);
    final_pose = Eigen::Vector3d(x_trajectory[x_trajectory.size() - 1], y_trajectory[y_trajectory.size() - 1], theta_trajectory[theta_trajectory.size() - 1]);
	BT_Util::put_in_blackboard(blackboard_, "desired_pose", desired_pose);

	Eigen::Vector2d command;
	
	double dist_err = sqrt(((final_pose.x() - current_x) * (final_pose.x() - current_x) + (final_pose.y() - current_y) * (final_pose.y() - current_y)));

	Eigen::Vector3d goal;

	if (dist_err < threshold_xy) {
		goal = final_pose;
		command = pd_control_ptr_->theta_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), goal);
	} else {
		goal = desired_pose;
		command = pd_control_ptr_->tank_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), goal, backwards);
	}
	// Eigen::Vector3d error = goal - tank_model_ptr_->getWorldPose();
	// error.z() = ghost_util::SmallestAngleDistRad(goal.z(), tank_model_ptr_->getWorldPose().z());
	// publishErrorPose(error);
	std::cout << "des angle: " << goal.z() << std::endl;
	
	auto fwd_cmd = ghost_util::clamp(command[0], -max_speed_linear, max_speed_linear);
	auto turn_cmd = ghost_util::clamp(command[1], -max_speed_angular, max_speed_angular);

	BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
	BT_Util::put_in_blackboard(blackboard_, "turn_cmd", turn_cmd);

	tank_model_ptr_->driveCommand(fwd_cmd, turn_cmd);
}

void MoveToPoseBoomerang::publishTrajectoryVisualization()
{
	visualization_msgs::msg::MarkerArray msg{};

	double search_radius = BT_Util::get_input<double>(this, "search_radius_m");
	
	Eigen::Vector3d desired_pose;
	BT_Util::get_from_blackboard(blackboard_, "desired_pose", desired_pose);

	visualization_msgs::msg::Marker search_radius_marker{};
	search_radius_marker.header.frame_id = "base_link";
	search_radius_marker.header.stamp = node_ptr_->get_clock()->now();
	search_radius_marker.id = 1;
	search_radius_marker.type = 3;   // cylinder type
	search_radius_marker.action = 0;
	search_radius_marker.scale.x = 2 * search_radius;
	search_radius_marker.scale.y = 2 * search_radius;
	search_radius_marker.scale.z = 0.01;
	search_radius_marker.color.b = 1.0;
	search_radius_marker.color.a = 0.3;

	visualization_msgs::msg::Marker carrot{};
	carrot.header.frame_id = "map";
	carrot.header.stamp = node_ptr_->get_clock()->now();
	carrot.id = 2;
	carrot.type = 4;   // line type
	carrot.action = 0;
	carrot.scale.x = 0.01;
	carrot.scale.y = 1.0;
	carrot.scale.z = 1.0;
	carrot.color.g = 1.0;
	carrot.color.a = 0.5;
	geometry_msgs::msg::Point p_robot;
	p_robot.x = tank_model_ptr_->getWorldPose().x();
	p_robot.y = tank_model_ptr_->getWorldPose().y();
	p_robot.z = 0.0;
	geometry_msgs::msg::Point p_carrot;
	p_carrot.set__x(desired_pose.x());
	p_carrot.set__y(desired_pose.y());
	p_carrot.z = 0.0;
	carrot.points.push_back(p_robot);
	carrot.points.push_back(p_carrot);

	visualization_msgs::msg::Marker marker{};
	marker.header.frame_id = "map";
	marker.header.stamp = node_ptr_->get_clock()->now();
	marker.id = 0;
	marker.type = 8;   // points type
	marker.action = 0;
	marker.scale.x = 0.025;
	marker.scale.y = 0.025;
	marker.scale.z = 0.1;
	marker.color.r = 1.0;
	marker.color.a = 1.0;

	for (int i = 0; i < robot_trajectory_.x_trajectory.position_vector.size(); i += 5) {
		geometry_msgs::msg::Point p;
		p.x = robot_trajectory_.x_trajectory.position_vector[i];
		p.y = robot_trajectory_.y_trajectory.position_vector[i];
		p.z = 0.0;
		marker.points.push_back(p);
	}
	msg.markers.push_back(search_radius_marker);
	msg.markers.push_back(carrot);
	msg.markers.push_back(marker);

	BT_Util::get_from_blackboard(blackboard_, "trajectory_viz_pub", trajectory_viz_pub_);
	trajectory_viz_pub_->publish(msg);
}

} // namespace ghost_tank
