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

#include "ghost_tank/bt_nodes/intakeCmd.hpp"

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
IntakeCmd::IntakeCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList IntakeCmd::providedPorts()
{
  // This action has a single input port called "message"
  return {
    BT::InputPort<bool>("auto"),
    BT::InputPort<bool>("ground_intake"),
  };
}

BT::NodeStatus IntakeCmd::tick()
{
  bool automatic = BT_Util::get_input<bool>(this, "auto");
  bool ground_intake = BT_Util::get_input<bool>(this, "ground_intake");

  bool target_red = false;
  BT_Util::get_from_blackboard(blackboard_, "target_red", target_red);

  BT_Util::put_in_blackboard(blackboard_, "want_red", target_red);
  BT_Util::put_in_blackboard(blackboard_, "ring_detector_active", automatic);
  BT_Util::put_in_blackboard(blackboard_, "ground_intake_active", ground_intake);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
