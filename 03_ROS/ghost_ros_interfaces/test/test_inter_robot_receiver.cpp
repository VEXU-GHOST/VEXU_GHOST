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

// Integration test for the inter-robot receiver node. Publishes a peer OtherRobot packet on
// /comms/other_robot and verifies the node broadcasts the de-quantized map->other_robot/base_link TF.

#include <chrono>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ghost_msgs/msg/other_robot.hpp>
#include "ghost_ros_interfaces/comms/inter_robot_receiver_node.hpp"

using ghost_ros_interfaces::InterRobotReceiverNode;
using namespace std::chrono_literals;

class InterRobotReceiverTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("inter_robot_receiver_test_helper");
    peer_pub_ = test_node_->create_publisher<ghost_msgs::msg::OtherRobot>(
      "comms/other_robot", rclcpp::SensorDataQoS());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(test_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void TearDown() override
  {
    tf_listener_.reset();
    tf_buffer_.reset();
    peer_pub_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  // Spins the receiver + test node, re-publishing the peer packet each cycle, until the peer TF is
  // available or timeout. Returns the looked-up transform.
  bool waitForPeerTransform(
    const std::shared_ptr<InterRobotReceiverNode> & node,
    const ghost_msgs::msg::OtherRobot & peer,
    geometry_msgs::msg::TransformStamped & out,
    const std::string & map_frame = "map",
    const std::string & child_frame = "other_robot/base_link",
    std::chrono::seconds timeout = 10s)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(test_node_);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
      peer_pub_->publish(peer);
      executor.spin_some(50ms);
      if (tf_buffer_->canTransform(map_frame, child_frame, tf2::TimePointZero)) {
        out = tf_buffer_->lookupTransform(map_frame, child_frame, tf2::TimePointZero);
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<ghost_msgs::msg::OtherRobot>::SharedPtr peer_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// A normal peer packet is de-quantized into the map->other_robot/base_link transform.
TEST_F(InterRobotReceiverTestFixture, broadcastsPeerPose) {
  auto node = std::make_shared<InterRobotReceiverNode>();

  ghost_msgs::msg::OtherRobot peer{};
  peer.x = 20;       // 20 * 0.05 = 1.0 m
  peer.y = 40;       // 40 * 0.05 = 2.0 m
  peer.theta = 64;   // 64 / 256 * 2pi = pi/2
  peer.status = ghost_msgs::msg::OtherRobot::STATUS_SCORING;

  geometry_msgs::msg::TransformStamped tf;
  ASSERT_TRUE(waitForPeerTransform(node, peer, tf));

  EXPECT_NEAR(tf.transform.translation.x, 1.0, 1e-3);
  EXPECT_NEAR(tf.transform.translation.y, 2.0, 1e-3);
  double yaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-3);
}

// The all-zero packet (signature of an RHI slot that never received peer data) must NOT produce a
// bogus (0, 0) transform.
TEST_F(InterRobotReceiverTestFixture, ignoresStaleSlot) {
  auto node = std::make_shared<InterRobotReceiverNode>();

  ghost_msgs::msg::OtherRobot peer{};  // every field zero

  geometry_msgs::msg::TransformStamped tf;
  EXPECT_FALSE(waitForPeerTransform(node, peer, tf, "map", "other_robot/base_link", 2s));
}

// A real pose with status still STATUS_UNKNOWN (producer running before any behavior sets status)
// must still broadcast: status drives game logic, not whether the pose is valid.
TEST_F(InterRobotReceiverTestFixture, broadcastsUnknownStatusPose) {
  auto node = std::make_shared<InterRobotReceiverNode>();

  ghost_msgs::msg::OtherRobot peer{};
  peer.x = 10;  // 10 * 0.05 = 0.5 m -> non-zero, so not the stale slot
  peer.status = ghost_msgs::msg::OtherRobot::STATUS_UNKNOWN;
  peer.seq = 3;

  geometry_msgs::msg::TransformStamped tf;
  ASSERT_TRUE(waitForPeerTransform(node, peer, tf));
  EXPECT_NEAR(tf.transform.translation.x, 0.5, 1e-3);
}

// A peer announcing a mismatched protocol version is distrusted: later poses are ignored.
TEST_F(InterRobotReceiverTestFixture, ignoresIncompatiblePeer) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"protocol_version", 1}});
  auto node = std::make_shared<InterRobotReceiverNode>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node_);

  // Phase 1: advertise a mismatched version so the peer is marked incompatible.
  ghost_msgs::msg::OtherRobot version{};
  version.status = ghost_msgs::msg::OtherRobot::STATUS_VERSION;
  version.target_x = 99;  // != our protocol_version (1)
  auto t0 = std::chrono::steady_clock::now();
  while (rclcpp::ok() && (std::chrono::steady_clock::now() - t0) < 1500ms) {
    peer_pub_->publish(version);
    executor.spin_some(50ms);
    std::this_thread::sleep_for(20ms);
  }

  // Phase 2: a normal pose must now be ignored (no TF broadcast).
  ghost_msgs::msg::OtherRobot peer{};
  peer.x = 20;
  peer.y = 40;
  peer.status = ghost_msgs::msg::OtherRobot::STATUS_SCORING;
  peer.seq = 5;
  auto t1 = std::chrono::steady_clock::now();
  while (rclcpp::ok() && (std::chrono::steady_clock::now() - t1) < 2s) {
    peer_pub_->publish(peer);
    executor.spin_some(50ms);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_FALSE(tf_buffer_->canTransform("map", "other_robot/base_link", tf2::TimePointZero));
}
