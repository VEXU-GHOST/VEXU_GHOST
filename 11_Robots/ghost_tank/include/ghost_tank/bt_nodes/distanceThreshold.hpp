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

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ghost_tank/tank_tree.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "ghost_msgs/msg/distance_sensor_state.hpp"

namespace ghost_tank
{

// Condition node that succeeds while a named distance sensor reads within an
// inclusive [min_mm, max_mm] window. The sensor is selected at runtime via the
// "sensor_name" port (e.g. "left", "right"), which maps to /sensors/distance/<name>.
class DistanceThreshold : public BT::SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature
  DistanceThreshold(const std::string & name, const BT::NodeConfig & config);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  BT::NodeStatus tick();

  void distanceUpdate(const ghost_msgs::msg::DistanceSensorState::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp::Subscription<ghost_msgs::msg::DistanceSensorState>::SharedPtr distance_sub_;

  // Sensor the subscription is currently bound to; "" until the first tick. The
  // subscription is (re)created lazily when the requested sensor_name changes.
  std::string subscribed_sensor_;

  uint16_t distance_mm_ = 0;
  uint8_t range_status_ = 1;   // non-zero == invalid until a good reading arrives
  bool have_reading_ = false;
};

} // ghost_tank
