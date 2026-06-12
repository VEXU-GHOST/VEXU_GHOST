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

#include "ghost_tank/bt_nodes/distanceThreshold.hpp"

namespace ghost_tank
{

using std::placeholders::_1;

DistanceThreshold::DistanceThreshold(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

void DistanceThreshold::distanceUpdate(const ghost_msgs::msg::DistanceSensorState::SharedPtr msg)
{
  distance_mm_ = msg->distance_mm;
  range_status_ = msg->range_status;
  have_reading_ = true;
}

BT::PortsList DistanceThreshold::providedPorts()
{
  return {
    BT::InputPort<std::string>("sensor_name"),  // distance sensor name, e.g. "left"
    BT::InputPort<double>("min_mm"),            // lower bound (inclusive), millimeters
    BT::InputPort<double>("max_mm"),            // upper bound (inclusive), millimeters
  };
}

BT::NodeStatus DistanceThreshold::tick()
{
  std::string sensor_name = BT_Util::get_input<std::string>(this, "sensor_name");
  double min_mm = BT_Util::get_input<double>(this, "min_mm");
  double max_mm = BT_Util::get_input<double>(this, "max_mm");

  // (Re)bind the subscription the first time we see a sensor name, or whenever it
  // changes. The publisher uses SensorDataQoS (best effort), so match it here or
  // no samples are delivered.
  if (sensor_name != subscribed_sensor_) {
    distance_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::DistanceSensorState>(
      "/sensors/distance/" + sensor_name, rclcpp::SensorDataQoS(),
      std::bind(&DistanceThreshold::distanceUpdate, this, _1));
    subscribed_sensor_ = sensor_name;
    have_reading_ = false;
  }

  // No sample yet, or the sensor flagged the last measurement invalid.
  if (!have_reading_ || range_status_ != 0) {
    return BT::NodeStatus::FAILURE;
  }

  if (distance_mm_ >= min_mm && distance_mm_ <= max_mm) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

} // ghost_tank
