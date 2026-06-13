/*
 *   Copyright (c) 2025 Karmanyaah Malhotra
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

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "behaviortree_cpp/behavior_tree.h"

#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_tank/control/velocity_controller.hpp"
#include "ghost_tank/tank_model.hpp"

namespace ghost_tank
{

// Open-loop-in-time, closed-loop-in-velocity drive primitive: holds a fixed
// chassis velocity setpoint -- linear x (m/s) and angular ang_z (rad/s) -- for
// timeout_ms, then stops and returns SUCCESS. Velocity is tracked by the shared
// VelocityController (PD on velocity error + velocity feedforward (kV) + static
// feedforward (kS)); its gains live in the robot config (velocity_linear / velocity_angular)
// and are loaded once in TankRobotPlugin, exactly like the arc_turn controller.
// This node only supplies the setpoint and the duration.
class MoveVelocityPDFF : public BT::StatefulActionNode
{
public:
  MoveVelocityPDFF(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void stopMotors();
  // Publishes the raw velocity setpoint (x_, ang_z_) as a Twist on /nav2/cmd_vel.
  // Additive to the PDFF drive below -- purely for downstream relay/visualization.
  void publishCmdVel();

  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::shared_ptr<VelocityController> velocity_controller_ptr_;
  BT::Blackboard::Ptr blackboard_;

  // Publisher for the commanded velocity setpoint on /nav2/cmd_vel.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_ptr_;

  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> prev_time_;

  // Velocity setpoint and duration, read on each onStart().
  double x_{0.0};        // commanded linear velocity, m/s
  double ang_z_{0.0};    // commanded angular velocity, rad/s
  int timeout_ms_{0};
};

} // namespace ghost_tank
