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

#include <chrono>
#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_tree.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_msgs/msg/distance_sensor_state.hpp"

namespace ghost_tank
{

// Opens the clamp, then closes it as soon as the auto_claw distance sensor reads
// below close_mm (same behavior as the teleop auto-close in ClydePlugin). If the
// sensor never triggers, the clamp closes after timeout_s. Always returns SUCCESS
// with the clamp closed.
class AutoClawCmd : public BT::StatefulActionNode
{
public:
  AutoClawCmd(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void distanceUpdate(const ghost_msgs::msg::DistanceSensorState::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp::Subscription<ghost_msgs::msg::DistanceSensorState>::SharedPtr distance_sub_;

  uint16_t distance_mm_ = 0;
  bool valid_ = false;
  std::chrono::steady_clock::time_point msg_time_{};
  std::chrono::steady_clock::time_point start_time_{};
};

} // ghost_tank
