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
#include <ghost_tank/motion_planning/trajectory.hpp>

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
class FollowPath : public BT::StatefulActionNode
{
public:
  // If your Node has ports, you must use this constructor signature
  FollowPath(const std::string & name, const BT::NodeConfig & config);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart();

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning();

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted();

private:
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<motion_planning::Trajectory> tank_trajectory_ptr_;
  BT::Blackboard::Ptr blackboard_;

  double xy_exit_threshold_m_{0.0};
  double angle_exit_threshold_rad_{0.0};
  double lin_vel_exit_threshold_mps_{0.0};
  double ang_vel_exit_threshold_radps_{0.0};
  double max_speed_linear_percent_{0.0};
  double max_speed_angular_percent_{0.0};
  int timeout_ms_{0};
  bool use_theta_{false};

  motion_planning::Trajectory trajectory_;
};

} // namespace ghost_tank {
