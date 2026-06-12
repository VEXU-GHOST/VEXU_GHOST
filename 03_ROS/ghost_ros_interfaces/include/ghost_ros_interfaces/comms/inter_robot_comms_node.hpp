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

#pragma once

// The ROS end of inter-robot comms, both halves in one node:
//
//   produce: quantize this robot's map->base_link pose + status -> /comms/self  (the V5 serial node
//            relays it to the peer over VEXlink)
//   consume: /comms/other_robot (the peer's relayed state) -> de-quantize -> broadcast a
//            map -> other_robot/base_link TF for rviz / costmaps / tf lookups
//
// V1 scope: position only. target_x/target_y are left at 0 (intent is a follow-up), and status comes
// from an optional /comms/self_status topic (default STATUS_UNKNOWN) so the behavior layer can drive
// it without this node depending on the strategy stack. A STATUS_VERSION packet carrying the protocol
// version in target_x is interleaved at a low rate; on the consume side a mismatched version marks the
// peer incompatible and its pose is ignored. A fully-zero packet (the never-written RHI slot) is also
// ignored so we never broadcast a bogus (0, 0) pose.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <ghost_msgs/msg/other_robot.hpp>

namespace ghost_ros_interfaces
{

class InterRobotCommsNode : public rclcpp::Node
{
public:
  explicit InterRobotCommsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("inter_robot_comms_node", options)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    peer_base_frame_ = declare_parameter("peer_base_frame", "other_robot/base_link");
    position_resolution_m_ = declare_parameter("position_resolution_m", 0.05);
    protocol_version_ = declare_parameter("protocol_version", 1);
    double publish_rate_hz = declare_parameter("publish_rate_hz", 10.0);
    double version_rate_hz = declare_parameter("version_rate_hz", 0.5);
    version_period_s_ = (version_rate_hz > 0.0) ? (1.0 / version_rate_hz) : 0.0;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Produce
    self_pub_ = create_publisher<ghost_msgs::msg::OtherRobot>(
      "comms/self", rclcpp::SensorDataQoS());
    status_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "comms/self_status", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {status_ = msg->data;});
    last_version_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&InterRobotCommsNode::publishSelf, this));

    // Consume
    peer_sub_ = create_subscription<ghost_msgs::msg::OtherRobot>(
      "comms/other_robot", rclcpp::SensorDataQoS(),
      std::bind(&InterRobotCommsNode::onOtherRobot, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Inter-robot comms: %s->%s -> /comms/self at %.1f Hz; /comms/other_robot -> TF %s->%s",
      map_frame_.c_str(), base_frame_.c_str(), publish_rate_hz,
      map_frame_.c_str(), peer_base_frame_.c_str());
  }

private:
  ////////////////////////// Produce //////////////////////////

  // Sets the heartbeat counter and publishes. seq only advances on a real publish so the receiver can
  // use seq gaps to estimate link loss (skipped cycles must not look like dropped packets).
  void publish(ghost_msgs::msg::OtherRobot & msg)
  {
    msg.seq = seq_++;
    self_pub_->publish(msg);
  }

  void publishSelf()
  {
    auto current_time = now();

    // Interleave a low-rate version announcement (target_x carries the protocol version).
    if (version_period_s_ > 0.0 &&
      (current_time - last_version_time_).seconds() >= version_period_s_)
    {
      ghost_msgs::msg::OtherRobot version_msg{};
      version_msg.status = ghost_msgs::msg::OtherRobot::STATUS_VERSION;
      version_msg.target_x = static_cast<uint8_t>(protocol_version_ & 0xFF);
      last_version_time_ = current_time;
      publish(version_msg);
      return;
    }

    // Normal packet: quantized map->base_link pose + current status.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No %s->%s transform (%s); skipping inter-robot pose update.",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    double yaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);

    ghost_msgs::msg::OtherRobot msg{};
    msg.x = quantizePosition(tf.transform.translation.x);
    msg.y = quantizePosition(tf.transform.translation.y);
    msg.theta = quantizeTheta(yaw);
    msg.status = status_;
    // target_x / target_y intentionally left 0 (intent is a follow-up).
    publish(msg);
  }

  ////////////////////////// Consume //////////////////////////

  void onOtherRobot(const ghost_msgs::msg::OtherRobot::SharedPtr msg)
  {
    // Version announcement: no pose, just a compatibility check against our own protocol version.
    if (msg->status == ghost_msgs::msg::OtherRobot::STATUS_VERSION) {
      peer_compatible_ = (static_cast<int>(msg->target_x) == protocol_version_);
      if (!peer_compatible_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Peer protocol version %d != ours %d; ignoring peer state.",
          static_cast<int>(msg->target_x), protocol_version_);
      }
      return;
    }

    // Don't trust a peer we know is on a mismatched build.
    if (!peer_compatible_) {
      return;
    }

    // Ignore the zero-initialized "no peer data yet" slot (every field zero). A real packet advances
    // seq, so this only suppresses a slot that has never been written by the peer.
    const bool stale_slot = (msg->status == ghost_msgs::msg::OtherRobot::STATUS_UNKNOWN) &&
      (msg->x == 0) && (msg->y == 0) && (msg->theta == 0) &&
      (msg->target_x == 0) && (msg->target_y == 0) && (msg->seq == 0);
    if (stale_slot) {
      return;
    }

    const double x = msg->x * position_resolution_m_;
    const double y = msg->y * position_resolution_m_;
    const double yaw = static_cast<double>(msg->theta) / 256.0 * 2.0 * M_PI;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = peer_base_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(yaw / 2.0);
    tf.transform.rotation.w = std::cos(yaw / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }

  ////////////////////////// Quantization //////////////////////////

  // Meters -> 5 cm units, clamped to the uint8 range. Assumes a non-negative map frame (corner origin).
  uint8_t quantizePosition(double meters) const
  {
    double units = std::round(meters / position_resolution_m_);
    units = std::clamp(units, 0.0, 255.0);
    return static_cast<uint8_t>(units);
  }

  // Radians -> 0..255 over [0, 2*pi).
  uint8_t quantizeTheta(double yaw_rad) const
  {
    double wrapped = std::fmod(yaw_rad, 2.0 * M_PI);
    if (wrapped < 0.0) {
      wrapped += 2.0 * M_PI;
    }
    long units = std::lround(wrapped / (2.0 * M_PI) * 256.0);
    return static_cast<uint8_t>(units % 256);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string peer_base_frame_;
  double position_resolution_m_;
  int protocol_version_;
  double version_period_s_;

  uint8_t status_{ghost_msgs::msg::OtherRobot::STATUS_UNKNOWN};
  uint8_t seq_{0};
  rclcpp::Time last_version_time_;
  bool peer_compatible_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<ghost_msgs::msg::OtherRobot>::SharedPtr self_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_sub_;
  rclcpp::Subscription<ghost_msgs::msg::OtherRobot>::SharedPtr peer_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace ghost_ros_interfaces
