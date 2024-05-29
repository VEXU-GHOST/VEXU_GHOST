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

#include "ghost_swerve/bt_nodes/loggingNode.hpp"

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
LoggingNode::LoggingNode(
  const std::string & name, const BT::NodeConfig & config,
  std::shared_ptr<rclcpp::Node> node_ptr)
: BT::SyncActionNode(name, config),
  node_ptr_(node_ptr)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList LoggingNode::providedPorts()
{
  // This action has a single input port called "message"
  return {
    BT::InputPort<std::string>("message")
  };
}

// Override the virtual function tick()
BT::NodeStatus LoggingNode::tick()
{
  BT::Expected<std::string> msg = getInput<std::string>("message");
  // Check if expected is valid. If not, throw its error
  if (!msg) {
    throw BT::RuntimeError(
            "missing required input [message]: ",
            msg.error() );
  }
  // use the method value() to extract the valid message.
  RCLCPP_INFO(node_ptr_->get_logger(), msg.value().c_str());
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return BT::NodeStatus::SUCCESS;
}
