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

#include <chrono>
#include <math.h>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "ghost_msgs/msg/ghost_robot_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class GroundTruthPosePublisher : public rclcpp::Node
{
public:
  GroundTruthPosePublisher()
  : Node("ground_truth_pose_publisher")
  {
    // Use simulated time in ROS
    rclcpp::Parameter use_sim_time_param("use_sim_time", true);
    this->set_parameter(use_sim_time_param);

    // Publishers
    world_tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf",
      10
    );

    robot_pose_pub_ = this->create_publisher<ghost_msgs::msg::GhostRobotState>(
      "/gz/model_pose",
      10
    );

    // Subscribers
    model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states",
      10,
      std::bind(&GroundTruthPosePublisher::modelStatesCallback, this, _1)
    );

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }

  void publishRobotPose(const float x, const float y, const float theta)
  {
    auto ghost_robot_state_msg = ghost_msgs::msg::GhostRobotState{};
    ghost_robot_state_msg.header.stamp = get_clock()->now();
    ghost_robot_state_msg.header.frame_id = "odom";

    ghost_robot_state_msg.x = x;
    ghost_robot_state_msg.y = y;
    ghost_robot_state_msg.theta = theta;

    robot_pose_pub_->publish(ghost_robot_state_msg);
  }

  void publishWorldTransform(
    const float x,
    const float y,
    const float z,
    const float q1,
    const float q2,
    const float q3,
    const float q4
  )
  {
    auto transform_msg = geometry_msgs::msg::TransformStamped{};

    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "odom";
    transform_msg.child_frame_id = "base_link";

    transform_msg.transform.translation.x = x;
    transform_msg.transform.translation.y = y;
    transform_msg.transform.translation.z = z;

    transform_msg.transform.rotation.w = q1;
    transform_msg.transform.rotation.x = q2;
    transform_msg.transform.rotation.y = q3;
    transform_msg.transform.rotation.z = q4;

    auto tf_msg = tf2_msgs::msg::TFMessage{};
    tf_msg.transforms.push_back(transform_msg);

    world_tf_pub_->publish(tf_msg);
  }

  void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    int swerve_model_index = -1;
    for (int i = 0; i < msg->name.size(); i++) {
      if (msg->name[i] == "ghost1") {
        swerve_model_index = i;
      }
    }
    if (swerve_model_index == -1) {
      RCLCPP_WARN(this->get_logger(), "Swerve Gazebo Model not found in /ModelStates topic");
    } else {
      x_ = msg->pose[swerve_model_index].position.x;
      y_ = msg->pose[swerve_model_index].position.y;
      float z = msg->pose[swerve_model_index].position.z;
      float q1 = msg->pose[swerve_model_index].orientation.x;
      float q2 = msg->pose[swerve_model_index].orientation.x;
      float q3 = msg->pose[swerve_model_index].orientation.z;
      float q4 = msg->pose[swerve_model_index].orientation.w;
      theta_ = acos(1 - 2 * (q2 * q2 + q3 * q3));

      publishRobotPose(x_, y_, theta_);
      publishWorldTransform(x_, y_, z, q1, q2, q3, q4);
    }
  }

private:
  // Publishers
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr world_tf_pub_;
  rclcpp::Publisher<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_pose_pub_;

  // Subscriptions
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;

  // Member Variables
  float x_;
  float y_;
  float theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
