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

// Integration test for the inter-robot producer node. Broadcasts a fake map->base_link transform and
// verifies the node publishes a correctly-quantized OtherRobot packet on /comms/self.

#include <chrono>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ghost_msgs/msg/other_robot.hpp>
#include "ghost_ros_interfaces/comms/inter_robot_publisher_node.hpp"

using ghost_ros_interfaces::InterRobotPublisherNode;
using namespace std::chrono_literals;

class InterRobotPublisherTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("inter_robot_test_helper");
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);

    last_msg_.reset();
    sub_ = test_node_->create_subscription<ghost_msgs::msg::OtherRobot>(
      "comms/self", rclcpp::SensorDataQoS(),
      [this](const ghost_msgs::msg::OtherRobot::SharedPtr msg) {last_msg_ = msg;});
  }

  void TearDown() override
  {
    sub_.reset();
    static_broadcaster_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  // Publishes a static map->base_link transform with the given planar pose.
  void broadcastPose(double x, double y, double yaw)
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

  // Spins the producer node + test node until a packet matching the predicate arrives or timeout.
  bool spinUntilMsg(
    const std::shared_ptr<InterRobotPublisherNode> & node,
    std::function<bool(const ghost_msgs::msg::OtherRobot &)> predicate,
    std::chrono::seconds timeout = 10s)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(test_node_);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
      executor.spin_some(50ms);
      if (last_msg_ && predicate(*last_msg_)) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<ghost_msgs::msg::OtherRobot>::SharedPtr sub_;
  ghost_msgs::msg::OtherRobot::SharedPtr last_msg_;
};

// With a fake map->base_link transform, the node publishes the correctly-quantized pose.
TEST_F(InterRobotPublisherTestFixture, publishesQuantizedPose) {
  broadcastPose(1.0, 2.0, M_PI / 2.0);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"version_rate_hz", 0.0},          // disable version packets: every tick is a normal packet
    {"publish_rate_hz", 50.0},
    {"position_resolution_m", 0.05},
  });
  auto node = std::make_shared<InterRobotPublisherNode>(options);

  // Wait for a normal (non-version) packet.
  ASSERT_TRUE(
    spinUntilMsg(
      node, [](const ghost_msgs::msg::OtherRobot & m) {
        return m.status != ghost_msgs::msg::OtherRobot::STATUS_VERSION;
      }));

  // 1.0 m / 0.05 = 20, 2.0 m / 0.05 = 40, (pi/2) / (2*pi) * 256 = 64.
  EXPECT_EQ(last_msg_->x, 20);
  EXPECT_EQ(last_msg_->y, 40);
  EXPECT_EQ(last_msg_->theta, 64);
  EXPECT_EQ(last_msg_->status, ghost_msgs::msg::OtherRobot::STATUS_UNKNOWN);
  EXPECT_EQ(last_msg_->target_x, 0);  // intent not sent in V1
  EXPECT_EQ(last_msg_->target_y, 0);
}

// A position at the field origin quantizes to zero; a different yaw quantizes correctly.
TEST_F(InterRobotPublisherTestFixture, publishesOriginPose) {
  broadcastPose(0.0, 0.0, M_PI);  // pi -> 128

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"version_rate_hz", 0.0},
    {"publish_rate_hz", 50.0},
  });
  auto node = std::make_shared<InterRobotPublisherNode>(options);

  ASSERT_TRUE(
    spinUntilMsg(
      node, [](const ghost_msgs::msg::OtherRobot & m) {
        return m.status != ghost_msgs::msg::OtherRobot::STATUS_VERSION;
      }));

  EXPECT_EQ(last_msg_->x, 0);
  EXPECT_EQ(last_msg_->y, 0);
  EXPECT_EQ(last_msg_->theta, 128);
}

// The interleaved version packet carries the protocol version in target_x (no TF required).
TEST_F(InterRobotPublisherTestFixture, publishesVersionPacket) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"version_rate_hz", 1000.0},       // fire a version packet immediately
    {"publish_rate_hz", 50.0},
    {"protocol_version", 7},
  });
  auto node = std::make_shared<InterRobotPublisherNode>(options);

  ASSERT_TRUE(
    spinUntilMsg(
      node, [](const ghost_msgs::msg::OtherRobot & m) {
        return m.status == ghost_msgs::msg::OtherRobot::STATUS_VERSION;
      }));

  EXPECT_EQ(last_msg_->status, ghost_msgs::msg::OtherRobot::STATUS_VERSION);
  EXPECT_EQ(last_msg_->target_x, 7);
}
