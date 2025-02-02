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

#include <iostream>
#include <ghost_robot_plugin_examples/ghost_robot_plugin_example.hpp>
#include <pluginlib/class_list_macros.hpp>

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using std::placeholders::_1;

namespace ghost_robot_plugin_examples
{

GhostRobotPluginExample::GhostRobotPluginExample()
{
}

void GhostRobotPluginExample::initialize()
{
  std::cout << "initialize" << std::endl;
}

void GhostRobotPluginExample::onNewSensorData()
{
  std::cout << "onNewSensorData" << std::endl;
}

void GhostRobotPluginExample::disabled()
{
  std::cout << "disabled" << std::endl;
}

void GhostRobotPluginExample::autonomous(double current_time)
{
  std::cout << "Autonomous" << current_time << std::endl;
}

void GhostRobotPluginExample::teleop(double current_time)
{
  std::cout << "Teleop" << current_time << std::endl;
}

} // namespace ghost_robot_plugin_examples

PLUGINLIB_EXPORT_CLASS(
  ghost_robot_plugin_examples::GhostRobotPluginExample,
  ghost_ros_interfaces::V5RobotBase)
