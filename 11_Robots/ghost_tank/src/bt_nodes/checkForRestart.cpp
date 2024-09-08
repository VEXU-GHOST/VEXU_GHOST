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

#include "ghost_tank/bt_nodes/checkForRestart.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

// If your Node has ports, you must use this constructor signature
CheckForRestart::CheckForRestart(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<rclcpp::Node> node_ptr,
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr)
: BT::SyncActionNode(name, config),
  node_ptr_(node_ptr),
  robot_hardware_interface_ptr_(robot_hardware_interface_ptr)
{
  start_time_ = std::chrono::system_clock::now();
  restarted_ = true;
}

// It is mandatory to define this STATIC method.
BT::PortsList CheckForRestart::providedPorts()
{
  return {
  };
}

// Override the virtual function tick()
BT::NodeStatus CheckForRestart::tick()
{
  if (restarted_) {
    restarted_ = false;
    start_time_ = std::chrono::system_clock::now();
  }

  auto now = std::chrono::system_clock::now();
  double time_elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
  if (time_elapsed > 500) {
    RCLCPP_INFO(node_ptr_->get_logger(), "BT reset");
    restarted_ = true;
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = std::chrono::system_clock::now();

  return BT::NodeStatus::SUCCESS;
}

} // namespace ghost_tank
