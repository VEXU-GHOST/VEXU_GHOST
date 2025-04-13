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

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
ConveyorCmd::ConveyorCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList ConveyorCmd::providedPorts()
{
  // This action has a single input port called "message"
  return {
  };
}

BT::NodeStatus ConveyorCmd::tick()
{
  // double timeout = BT_Util::get_input<double>(this, "timeout");
  // if (start_time_ == 0.0) {
  // BT_Util::get_from_blackboard(blackboard_, "auton_time_elapsed", start_time_);
  // }
  // double current_time = 0.0;
  // BT_Util::get_from_blackboard(blackboard_, "auton_time_elapsed", current_time);
  // if (current_time - start_time_ > timeout) {
  // return BT::NodeStatus::SUCCESS;
  // }

  std::unordered_map<std::string, int> digital_io_port_map;
  BT_Util::get_from_blackboard(blackboard_, "digital_io_port_map", digital_io_port_map);
  bool goal_detected = rhi_ptr_->getDigitalIOValue(digital_io_port_map["goal_rush_sensor"]);
  if (goal_detected) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

} // ghost_tank
