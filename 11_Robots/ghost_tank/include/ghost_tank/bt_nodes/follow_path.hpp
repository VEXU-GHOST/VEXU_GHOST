/*
 *   Copyright (c) 2025 Jake Wendling, Maxx Wilson
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

#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include <ghost_tank/control/trajectory.hpp>
#include <ghost_tank/control/tank_pid_controller.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
class FollowPath : public BT::StatefulActionNode
{
public:
  // If your Node has ports, you must use this constructor signature
  FollowPath(const std::string & name, const BT::NodeConfig & config);

  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart() override;

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning() override;

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted() override;

  virtual Eigen::Vector2d calculateControllerCommand() = 0;
  virtual void populateVisualizationMarkers() {}

  static BT::PortsList getBaseInputPorts();

protected:
  bool checkEndConditions();
  void normalizeControllerCommand();
  void updateVisualization();
  void publishExitThresholds();

  void updateCurrentState();

  std::shared_ptr<rclcpp::Node> node_ptr_;
  BT::Blackboard::Ptr blackboard_;

  // Visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_viz_pub_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_pub_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr exit_threshold_viz_pub_ptr_;
  visualization_msgs::msg::MarkerArray viz_msg_;


  // Tank Drive Control
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<motion_planning::Trajectory> tank_trajectory_ptr_;
  motion_planning::Trajectory trajectory_;
  std::shared_ptr<ghost_control::PIDController> m_distance_approach_controller_ptr;
  std::shared_ptr<ghost_control::PIDController> m_steering_approach_controller_ptr;
  std::shared_ptr<ghost_control::PIDController> m_distance_settling_controller_ptr;
  std::shared_ptr<ghost_control::PIDController> m_steering_settling_controller_ptr;
  
  double fwd_command_{0.0};
  double turn_command_{0.0};

  // Path Config
  double xy_exit_threshold_m_{0.0};
  double angle_exit_threshold_rad_{0.0};
  double lin_vel_exit_threshold_mps_{0.0};
  double ang_vel_exit_threshold_radps_{0.0};
  double max_speed_linear_percent_{0.0};
  double max_speed_angular_percent_{0.0};
  int timeout_ms_{0};
  bool use_theta_{false};


  Eigen::Vector3d goal_pose_;
  Eigen::Vector2d current_position_;
  double current_robot_theta_;

  // State Transition Handling
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  bool first_loop_{true};
  bool settling_{false};
  bool backwards_{false};
};

} // namespace ghost_tank
