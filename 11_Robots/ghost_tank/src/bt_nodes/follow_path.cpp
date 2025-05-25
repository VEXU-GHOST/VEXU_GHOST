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


#include "ghost_tank/bt_nodes/follow_path.hpp"

namespace ghost_tank
{

FollowPath::FollowPath(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_trajectory_ptr", tank_trajectory_ptr_);
}

BT::PortsList FollowPath::providedPorts()
{
  return {
    BT::InputPort<double>("xy_exit_threshold_tiles"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<double>("lin_vel_exit_threshold_tps", 1000.0, ""),
    BT::InputPort<double>("ang_vel_exit_threshold_dps", 1000.0, ""),
    BT::InputPort<double>("max_speed_linear_percent"),
    BT::InputPort<double>("max_speed_angular_percent"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<bool>("use_theta"),
  };
}

BT::NodeStatus FollowPath::onStart()
{
  // Get Blackboard Inputs
  xy_exit_threshold_m_ = BT_Util::get_input<double>(this, "xy_exit_threshold_tiles") * ghost_util::TILES_TO_METERS;
  angle_exit_threshold_rad_ = BT_Util::get_input<double>(this, "angle_exit_threshold_deg") * ghost_util::DEG_TO_RAD;
  lin_vel_exit_threshold_mps_ = BT_Util::get_input<double>(this, "lin_vel_exit_threshold_tiles") * ghost_util::TILES_TO_METERS;
  ang_vel_exit_threshold_radps_ = BT_Util::get_input<double>(this, "ang_vel_exit_threshold_dps") * ghost_util::DEG_TO_RAD;
  max_speed_linear_percent_ = BT_Util::get_input<double>(this, "max_speed_linear_percent");
  max_speed_angular_percent_ = BT_Util::get_input<double>(this, "max_speed_angular_percent");
  timeout_ms_ = BT_Util::get_input<int>(this, "timeout_ms");
  use_theta_ = BT_Util::get_input<bool>(this, "use_theta");

  // Update local trajectory copy
  trajectory_ = *tank_trajectory_ptr_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPath::onRunning()
{
//   return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}

void FollowPath::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank {
