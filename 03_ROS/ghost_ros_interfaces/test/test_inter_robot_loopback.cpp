/*
 *   Copyright (c) 2026 Karmanyaah Malhotra
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

// Whole-stack (minus V5 firmware + radio) integration test. Exercises the full ROS round trip:
//
//   fake map->base_link TF  -->  InterRobotPublisherNode  -->  /comms/self
//        --[loopback standing in for the V5 brain + VEXlink + peer]-->  /comms/other_robot
//        -->  InterRobotReceiverNode  -->  TF map->other_robot/base_link
//
// and asserts the peer pose that comes out the far end matches the pose that went in, within one
// quantization step. This is the pre-merge end-to-end check for everything but the hardware hop.

#include <chrono>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <ghost_msgs/msg/other_robot.hpp>
#include "ghost_ros_interfaces/comms/inter_robot_publisher_node.hpp"
#include "ghost_ros_interfaces/comms/inter_robot_receiver_node.hpp"

using ghost_ros_interfaces::InterRobotPublisherNode;
using ghost_ros_interfaces::InterRobotReceiverNode;
using namespace std::chrono_literals;

class InterRobotLoopbackTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("inter_robot_loopback_test_helper");
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);

    // Loopback: stand in for "A's brain -> VEXlink -> B's brain", which on B re-emits the packet on
    // /comms/other_robot. Here we just relay /comms/self straight to /comms/other_robot.
    peer_pub_ = test_node_->create_publisher<ghost_msgs::msg::OtherRobot>(
      "comms/other_robot", rclcpp::SensorDataQoS());
    self_sub_ = test_node_->create_subscription<ghost_msgs::msg::OtherRobot>(
      "comms/self", rclcpp::SensorDataQoS(),
      [this](const ghost_msgs::msg::OtherRobot::SharedPtr msg) {peer_pub_->publish(*msg);});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(test_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void TearDown() override
  {
    tf_listener_.reset();
    tf_buffer_.reset();
    self_sub_.reset();
    peer_pub_.reset();
    static_broadcaster_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  void broadcastSelfPose(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = test_node_->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(yaw / 2.0);
    tf.transform.rotation.w = std::cos(yaw / 2.0);
    static_broadcaster_->sendTransform(tf);
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<ghost_msgs::msg::OtherRobot>::SharedPtr peer_pub_;
  rclcpp::Subscription<ghost_msgs::msg::OtherRobot>::SharedPtr self_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

TEST_F(InterRobotLoopbackTestFixture, poseSurvivesFullRoundTrip) {
  const double in_x = 1.25, in_y = 3.40, in_yaw = M_PI;  // 25 / 68 / 128 units
  broadcastSelfPose(in_x, in_y, in_yaw);

  rclcpp::NodeOptions pub_options;
  pub_options.parameter_overrides(
  {
    {"version_rate_hz", 0.0},  // pose packets only, so every relayed packet carries a pose
    {"publish_rate_hz", 50.0},
  });
  auto publisher = std::make_shared<InterRobotPublisherNode>(pub_options);
  auto receiver = std::make_shared<InterRobotReceiverNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(receiver);
  executor.add_node(test_node_);

  geometry_msgs::msg::TransformStamped tf;
  bool got_tf = false;
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < 10s) {
    executor.spin_some(50ms);
    if (tf_buffer_->canTransform("map", "other_robot/base_link", tf2::TimePointZero)) {
      tf = tf_buffer_->lookupTransform("map", "other_robot/base_link", tf2::TimePointZero);
      got_tf = true;
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(got_tf) << "peer TF never arrived through the full producer->loopback->receiver chain";

  // Within one position quantum (5 cm) and one heading quantum (2*pi/256 ~= 0.0245 rad).
  EXPECT_NEAR(tf.transform.translation.x, in_x, 0.05);
  EXPECT_NEAR(tf.transform.translation.y, in_y, 0.05);
  double out_yaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
  EXPECT_NEAR(out_yaw, in_yaw, 2.0 * M_PI / 256.0 + 1e-6);
}
