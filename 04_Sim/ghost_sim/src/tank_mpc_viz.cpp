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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "ghost_msgs/msg/state_sol.hpp"

#include "ghost_sim/tank_mpc_viz.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace tank_mpc_viz
{

TankMPCViz::TankMPCViz()
: Node("tank_mpc_viz")
{

  // Use simulated time in ROS
  rclcpp::Parameter use_sim_time_param("use_sim_time", true);
  this->set_parameter(use_sim_time_param);

  // Publishers
  world_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
    "/tf",
    10
  );

  state_sol_sub_ = this->create_subscription<ghost_msgs::msg::StateSol>(
    "/tank_mpc_sol",
    10,
    std::bind(&TankMPCViz::stateSolCallback, this, _1)
  );

  timer_ = this->create_wall_timer(
    100ms, std::bind(&TankMPCViz::timerCallback, this));

  timer_index_ = 0;

}

void TankMPCViz::stateSolCallback(const ghost_msgs::msg::StateSol::SharedPtr msg)
{
  // Called when received new state solution from casadi
  // Iterate over state sol and populate tf_transforms_ vector

  for (int i = 0; i < msg->base_pose_x_traj.size(); i++) {

    geometry_msgs::msg::TransformStamped tf_stamped{};
    tf_stamped.header.stamp = this->get_clock()->now();
    tf_stamped.header.frame_id = "map";
    tf_stamped.child_frame_id = "base_link";
    tf_stamped.transform.translation.x = msg->base_pose_x_traj[i];

    tf_stamped.transform.translation.z = 0.0;

    tf_stamped.transform.rotation.z = msg->base_pose_tht_traj[0];

    tf_stamped.transform.translation.y = msg->base_pose_y_traj[i];
    tf_transforms_.emplace_back(tf_stamped);
    RCLCPP_INFO(get_logger(), "loop %d", i);

  }
  RCLCPP_INFO(get_logger(), "done stateSolCallback");
}

void TankMPCViz::timerCallback()
{
  if (tf_transforms_.size() != 0) {
    world_tf_pub_->publish(tf_transforms_.at(timer_index_));
    timer_index_++;
  } else if (timer_index_ >= tf_transforms_.size()) {
    timer_index_ = 0;
  }
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tank_mpc_viz::TankMPCViz>());
  rclcpp::shutdown();
  return 0;
}
