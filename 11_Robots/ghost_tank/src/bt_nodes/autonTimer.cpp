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

#include "ghost_tank/bt_nodes/autonTimer.hpp"

namespace ghost_tank
{

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
AutonTimer::AutonTimer(
  const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config){
    blackboard_ = config.blackboard;
	  blackboard_->get("node_ptr", node_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList AutonTimer::providedPorts()
{
  // This action has a single input port called "message"
  return {
    BT::InputPort<double>("seconds")
  };
}

void AutonTimer::halt()
{
  haltChild();
}

// Override the virtual function tick()
BT::NodeStatus AutonTimer::tick()
{
  // BT::Expected<double> seconds = getInput<double>("seconds");
  // // Check if expected is valid. If not, throw its error
  // if (!seconds) {
  //   throw BT::RuntimeError(
  //           "missing required input [message]: ",
  //           seconds.error() );
  // }
  double timeout = BT_Util::get_input<double>(this, "seconds");
  
  // double timeout = seconds.value();
  double time = blackboard_->get<double>("auton_time_elapsed");

  if (time > timeout) {
    RCLCPP_INFO(node_ptr_->get_logger(), "AutonTimeout: %f s passed", time);
    return BT::NodeStatus::FAILURE;
  }

  switch (child()->executeTick()) {
    case BT::NodeStatus::SUCCESS:
      haltChild();
      break;
    case BT::NodeStatus::FAILURE:
      haltChild();
      return BT::NodeStatus::FAILURE;
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    default:
      throw BT::LogicError("A child node must never return IDLE");
  }

  return BT::NodeStatus::SUCCESS;
}

} // ghost_tank
