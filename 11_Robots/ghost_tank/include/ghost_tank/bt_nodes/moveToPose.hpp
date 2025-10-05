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

#pragma once
#include <math.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_tank/control/tank_pid_controller.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

namespace ghost_tank
{

// abstract class for movetopose bt nodes
class MoveToPose : public BT::StatefulActionNode
{
public:
  // If your Node has ports, you must use this constructor signature
  MoveToPose(const std::string & name, const BT::NodeConfig & config);

  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart();

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning();

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted();

protected:
  std::shared_ptr<TankModel> tank_model_ptr_;
  rclcpp::Publisher<ghost_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> plan_time_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  BT::Blackboard::Ptr blackboard_;

  bool first_loop_;
  int past_index_;
  std::shared_ptr<TankPIDController> pd_control_ptr_;
  std::shared_ptr<TankPIDController> pd_control_threshold_ptr_;
  ghost_planners::RobotTrajectory robot_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curr_angle_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr des_angle_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fwd_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turn_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_cmd_pub;

  Eigen::Vector3d final_pose_ = Eigen::Vector3d::Zero();

  bool settling_{false};

  static constexpr double tile_to_meters = 0.6096;

  double xy_exit_threshold_m{0.0};
  double angle_exit_threshold_rad{0.0};
  double lin_vel_exit_threshold_mps{0.0};
  double ang_vel_exit_threshold_radps{0.0};
  double search_radius{0.0};
  double max_speed_linear_percent{0.0};
  double max_speed_angular_percent{0.0};
  bool use_theta{0};
  bool backwards{0};
  int timeout_ms{0};

  // gets all member variables from ports, must deal with mirrored also 
  virtual void GetBlackboardData() = 0;
  virtual void FirstLoop() = 0;
  virtual void GeneratePath() = 0;
  // virtual void GetFinalPose() = 0;
  void PurePursuit();
  bool CheckEndConditions(Eigen::Vector3d des_pos);
  void publishTrajectoryVisualization();

  void publishDrivetrainCommands(double fwd_cmd, double turn_cmd);

};

} // namespace ghost_tank {
