/*
 *   Copyright (c) 2024 Maxx Wilson
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

#include <ghost_util/angle_util.hpp>
#include "ghost_swerve/swerve_tree.hpp"


// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

namespace ghost_swerve
{

SwerveTree::SwerveTree(
  std::string bt_path,
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr,
  std::shared_ptr<SwerveModel> swerve_ptr,
  std::shared_ptr<rclcpp::Node> node_ptr,
  double burnout_absolute_rpm_threshold,
  double burnout_stall_duration_ms,
  double burnout_cooldown_duration_ms)
: bt_path_(bt_path),
  node_ptr_(node_ptr)
{
  BT::BehaviorTreeFactory factory;

  // add all nodes here
  factory.registerNodeType<LoggingNode>("Logging", node_ptr_);
  factory.registerNodeType<CheckForRestart>(
    "CheckForRestart", node_ptr_,
    robot_hardware_interface_ptr);
  factory.registerNodeType<MoveToPose>(
    "MoveToPose", node_ptr_, robot_hardware_interface_ptr,
    swerve_ptr);
  factory.registerNodeType<SwipeTail>(
    "SwipeTail", node_ptr_, robot_hardware_interface_ptr,
    swerve_ptr);
  factory.registerNodeType<IntakeCmd>(
    "IntakeCmd", node_ptr_, robot_hardware_interface_ptr, swerve_ptr,
    burnout_absolute_rpm_threshold,
    burnout_stall_duration_ms,
    burnout_cooldown_duration_ms);
  factory.registerNodeType<Climb>("Climb", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);
  // factory.registerNodeType<AutoDone>("AutoDone", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);

  tree_ = factory.createTreeFromFile(bt_path_);
}

void SwerveTree::tick_tree()
{
  tree_.tickExactlyOnce();
}

} // namespace ghost_swerve
