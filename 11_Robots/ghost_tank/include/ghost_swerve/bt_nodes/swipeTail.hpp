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

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/tank_model.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ghost_tank
{

class SwipeTail : public BT::StatefulActionNode
{
public:
  SwipeTail(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<rclcpp::Node> node_ptr,
    std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
    std::shared_ptr<tankModel> tank_ptr);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart();

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning();

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted();

private:
  template<typename T>
  T get_input(std::string key);

  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  std::shared_ptr<tankModel> tank_ptr_;
  bool started_;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
};

} // namespace ghost_tank
