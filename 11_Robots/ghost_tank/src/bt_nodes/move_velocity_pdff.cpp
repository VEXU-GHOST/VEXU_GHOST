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

#include <cmath>

#include "ghost_tank/bt_nodes/move_velocity_pdff.hpp"

namespace ghost_tank
{

MoveVelocityPDFF::MoveVelocityPDFF(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "velocity_controller_ptr", velocity_controller_ptr_);
}

BT::PortsList MoveVelocityPDFF::providedPorts()
{
  return {
    BT::InputPort<double>("x", 0.0, "commanded linear velocity, m/s"),
    BT::InputPort<double>("ang_z", 0.0, "commanded angular velocity, rad/s"),
    BT::InputPort<int>("timeout_ms", 1000, "how long to hold the velocity setpoint, ms")
  };
}

BT::NodeStatus MoveVelocityPDFF::onStart()
{
  start_time_ = std::chrono::system_clock::now();
  prev_time_ = start_time_;

  x_ = BT_Util::get_input<double>(this, "x");
  ang_z_ = BT_Util::get_input<double>(this, "ang_z");
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");

  velocity_controller_ptr_->reset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveVelocityPDFF::onRunning()
{
  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - start_time_).count();
  if (time_elapsed > std::abs(timeout_ms_)) {
    stopMotors();
    return BT::NodeStatus::SUCCESS;
  }

  // Commanded and measured chassis velocity, both as a normalized fraction of the
  // chassis max velocity. Measured linear speed is the world-twist magnitude and
  // measured angular is its z component, matching FollowPathControllerServerPDFF.
  double lin_cmd_frac = x_ / tank_model_ptr_->getMaxBaseLinearVelocity();
  double ang_cmd_frac = ang_z_ / tank_model_ptr_->getMaxBaseAngularVelocity();
  double lin_meas_frac = tank_model_ptr_->getWorldTwist().head<2>().norm() /
    tank_model_ptr_->getMaxBaseLinearVelocity();
  double ang_meas_frac = tank_model_ptr_->getWorldTwist().z() /
    tank_model_ptr_->getMaxBaseAngularVelocity();

  auto now = std::chrono::system_clock::now();
  double dt = std::chrono::duration<double>(now - prev_time_).count();
  prev_time_ = now;

  Eigen::Vector2d command = velocity_controller_ptr_->calculateCommand(
    lin_cmd_frac, lin_meas_frac, ang_cmd_frac, ang_meas_frac, dt);
  tank_model_ptr_->normalizeArcadeCommand(command);
  tank_model_ptr_->driveCommandArcade(command.x(), command.y());
  return BT::NodeStatus::RUNNING;
}

void MoveVelocityPDFF::stopMotors()
{
  tank_model_ptr_->driveCommandArcade(0.0, 0.0);
}

void MoveVelocityPDFF::onHalted()
{
  stopMotors();
  resetStatus();
}

} // namespace ghost_tank
