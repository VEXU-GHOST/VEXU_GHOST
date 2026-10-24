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

#include "ghost_tank/bt_nodes/autoClawCmd.hpp"

namespace ghost_tank
{

using std::placeholders::_1;
using namespace std::chrono_literals;

AutoClawCmd::AutoClawCmd(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);

  // Publisher uses SensorDataQoS (best effort), so match it here.
  std::string sensor_name = BT_Util::get_input<std::string>(this, "sensor_name", "auto_claw");
  distance_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::DistanceSensorState>(
    "/sensors/distance/" + sensor_name, rclcpp::SensorDataQoS(),
    std::bind(&AutoClawCmd::distanceUpdate, this, _1));
}

void AutoClawCmd::distanceUpdate(const ghost_msgs::msg::DistanceSensorState::SharedPtr msg)
{
  distance_mm_ = msg->distance_mm;
  valid_ = (msg->range_status == 0);
  msg_time_ = std::chrono::steady_clock::now();
}

BT::PortsList AutoClawCmd::providedPorts()
{
  return {
    BT::InputPort<std::string>("sensor_name", "auto_claw", "distance sensor name"),
    BT::InputPort<double>("close_mm", 75.0, "close when the sensor reads below this (mm)"),
    BT::InputPort<double>("timeout_s", 3.0, "close anyway after this many seconds"),
  };
}

BT::NodeStatus AutoClawCmd::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  // Open the clamp and wait for the sensor
  BT_Util::put_in_blackboard(blackboard_, "clamp_closed", false);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AutoClawCmd::onRunning()
{
  double close_mm = BT_Util::get_input<double>(this, "close_mm", 75.0);
  double timeout_s = BT_Util::get_input<double>(this, "timeout_s", 3.0);

  auto now = std::chrono::steady_clock::now();

  // Ignore readings older than 200 ms (sensor unplugged / host down)
  bool fresh = now - msg_time_ < 200ms;
  if (fresh && valid_ && distance_mm_ < close_mm) {
    std::cout << "[AutoClawCmd] auto-close at " << distance_mm_ << " mm" << std::endl;
    BT_Util::put_in_blackboard(blackboard_, "clamp_closed", true);
    return BT::NodeStatus::SUCCESS;
  }

  if (std::chrono::duration<double>(now - start_time_).count() > timeout_s) {
    std::cout << "[AutoClawCmd] timed out, closing" << std::endl;
    BT_Util::put_in_blackboard(blackboard_, "clamp_closed", true);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void AutoClawCmd::onHalted()
{
  // Leave the clamp as-is if the tree is halted mid-grab
}

} // ghost_tank
