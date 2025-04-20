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

#include "ghost_tank/bt_nodes/goalRushDetected.hpp"

namespace ghost_tank
{

using std::placeholders::_1;
// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
GoalRushDetected::GoalRushDetected(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config), goal_rush_l_(0), goal_rush_r_(0)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);

  goal_rush_proxmity_sub_l_ =
    node_ptr_->create_subscription<std_msgs::msg::Float32>(
    "/sensors/color_sensor/goal_rush_l/proximity", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {this->proxUpdate(msg, false);});

  goal_rush_proxmity_sub_r_ =
    node_ptr_->create_subscription<std_msgs::msg::Float32>(
    "/sensors/color_sensor/goal_rush_r/proximity", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {this->proxUpdate(msg, true);});

}

void GoalRushDetected::proxUpdate(const std_msgs::msg::Float32::SharedPtr v, bool right)
{
  if (right) {
    goal_rush_r_ = v->data;
  } else {
    goal_rush_l_ = v->data;
  }
}

// It is mandatory to define this STATIC method.
BT::PortsList GoalRushDetected::providedPorts()
{
  // This action has a single input port called "message"
  return {
    BT::InputPort<float>("threshold"),
    BT::InputPort<bool>("right"),
  };
}

BT::NodeStatus GoalRushDetected::tick()
{

  static int count = 0;
  float thresh = BT_Util::get_input<float>(this, "threshold");
  float val = BT_Util::get_input<bool>(this, "right") ? goal_rush_r_ : goal_rush_l_;

  if (val > thresh) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

} // ghost_tank
