/*
 *   Copyright (c) 2024 Melissa Cruz
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

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <chrono>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tank_mpc_viz
{

class TankMPCViz : public rclcpp::Node
{
public:
  /// Constructor
  TankMPCViz();

private:
  void stateSolCallback(const ghost_msgs::msg::StateSol::SharedPtr msg);
  void timerCallback();
  rclcpp::Subscription<ghost_msgs::msg::StateSol>::SharedPtr state_sol_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr world_tf_pub_;

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms_;

  // Current state published to rviz
  float x_ = 0.0;
  float y_ = 0.0;
  float theta_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_msgs::msg::TFMessage> tf_msg;
  int timer_index_;
};

}// namespace tank_mpc_viz
