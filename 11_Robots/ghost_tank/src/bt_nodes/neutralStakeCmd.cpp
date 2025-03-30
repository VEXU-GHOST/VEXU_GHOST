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

 #include "ghost_tank/bt_nodes/neutralStakeCmd.hpp"
 #include "ghost_tank/pdcontrol.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
NeutralStakeCmd::NeutralStakeCmd(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[NeutralStakeCmd::NeutralStakeCmd]" << std::endl;

  blackboard_ = config.blackboard;

  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList NeutralStakeCmd::providedPorts()
{
  return {
    BT::InputPort<int>("state"),
    BT::InputPort<double>("timeout"),
  };
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus NeutralStakeCmd::onStart()
{
  BT_Util::get_from_blackboard(blackboard_, "auton_time_elapsed", start_time_);
  return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void NeutralStakeCmd::onHalted()
{
  resetStatus();
}

BT::NodeStatus NeutralStakeCmd::onRunning()
{
  int state = BT_Util::get_input<int>(this, "state");
  double timeout = BT_Util::get_input<double>(this, "timeout");
  double current_time = 0.0;
  BT_Util::get_from_blackboard(blackboard_, "auton_time_elapsed", current_time);
  BT_Util::put_in_blackboard(blackboard_, "neutral_stake_pos", state);

  if (timeout > (current_time - start_time_)) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace ghost_tank
