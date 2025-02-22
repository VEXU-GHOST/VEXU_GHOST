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

#include "ghost_tank/bt_nodes/conveyorCmd.hpp"

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
    BT::InputPort<bool>("fwd"),
  };
}

BT::NodeStatus ConveyorCmd::tick()
{
  bool fwd = BT_Util::get_input<bool>(this, "fwd");
  
  // Conveyor control
  // We assume any manual conveyor control misaligns the hooks
  double conveyor_power = 0;
  int32_t conveyor_current = 0;
  if (fwd) {
    conveyor_power = 1.0;
    conveyor_current = 2500;
  } else {
    conveyor_power = -1.0;
    conveyor_current = 2500;
  }
  
  rhi_ptr_->setMotorVoltageCommandPercent("conveyor_motor", conveyor_power);
  rhi_ptr_->setMotorCurrentLimitMilliAmps("conveyor_motor", conveyor_current);

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
