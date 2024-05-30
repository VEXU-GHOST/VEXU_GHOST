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

#include "ghost_sim/bag_set_pose_time.hpp"

#include <memory>

using std::placeholders::_1;
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace bag_set_pose_time
{

BagSetPoseTime::BagSetPoseTime()
: Node("bag_set_pose_time")
{
  rclcpp::Parameter use_sim_time_param("use_sim_time", true);
  this->set_parameter(use_sim_time_param);
  // Subscribers
  // rviz_set_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  //    "/bag_set_pose_time",
  //    10,
  //    std::bind(&BagSetPoseTime::BagSetPoseTimeCallback, this, _1));
  rviz_set_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/bag_set_pose_time",
    10,
    std::bind(&BagSetPoseTime::BagSetPoseTimeCallback, this, _1));

  // Publishers
  set_pose_time_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initial_pose", 10);
}


void BagSetPoseTime::BagSetPoseTimeCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  auto sim_time_msg = geometry_msgs::msg::PoseWithCovarianceStamped{};
  sim_time_msg.header.stamp = this->get_clock()->now();
  sim_time_msg.header.frame_id = msg->header.frame_id;
  sim_time_msg.pose = msg->pose;

  set_pose_time_pub_->publish(sim_time_msg);
}

} // namespace bag_set_pose_time

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bag_set_pose_time::BagSetPoseTime>());
  rclcpp::shutdown();
  return 0;
}
