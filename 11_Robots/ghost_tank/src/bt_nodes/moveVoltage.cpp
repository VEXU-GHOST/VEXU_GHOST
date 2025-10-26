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


#include "ghost_tank/bt_nodes/moveVoltage.hpp"

namespace ghost_tank
{

MoveVoltage::MoveVoltage(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  first_loop_ = true;
}

BT::PortsList MoveVoltage::providedPorts()
{
  return {
    BT::InputPort<double>("forward_effort"),
    BT::InputPort<double>("angular_effort"),
    BT::InputPort<int>("timeout_ms")
  };
}

BT::NodeStatus MoveVoltage::onStart()
{
  first_loop_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveVoltage::onRunning()
{
  forward_effort = BT_Util::get_input<double>(this, "forward_effort");
  angular_effort = BT_Util::get_input<double>(this, "angular_effort");
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");

  if (first_loop_) {
    start_time_ = std::chrono::system_clock::now();
    first_loop_ = false;
  }

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > abs(timeout_ms)) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  tank_model_ptr_->driveCommandArcade(forward_effort, angular_effort);
  return BT::NodeStatus::RUNNING;
}

void MoveVoltage::onHalted()
{
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
  resetStatus();
}

} // namespace ghost_tank {
