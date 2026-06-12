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

// Producer side of inter-robot comms: builds this robot's OtherRobot packet and publishes it on
// /comms/self. JetsonV5SerialNode relays it to the V5 brain, which transmits it to the peer over
// VEXlink. The pose comes from the map->base_link transform, quantized to the OtherRobot wire format.
//
// V1 scope: position only. target_x/target_y are left at 0 (intent is a follow-up), and status is
// taken from an optional /comms/self_status topic (default STATUS_UNKNOWN) so the behavior layer can
// drive it without this node depending on the strategy stack. A STATUS_VERSION packet carrying the
// protocol version in target_x is interleaved at a low rate so a peer on a mismatched build can be
// detected.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ghost_msgs/msg/other_robot.hpp>

namespace ghost_ros_interfaces
{

class InterRobotPublisherNode : public rclcpp::Node
{
public:
  explicit InterRobotPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("inter_robot_publisher_node", options)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    position_resolution_m_ = declare_parameter("position_resolution_m", 0.05);
    protocol_version_ = declare_parameter("protocol_version", 1);
    double publish_rate_hz = declare_parameter("publish_rate_hz", 10.0);
    double version_rate_hz = declare_parameter("version_rate_hz", 0.5);
    version_period_s_ = (version_rate_hz > 0.0) ? (1.0 / version_rate_hz) : 0.0;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    self_pub_ = create_publisher<ghost_msgs::msg::OtherRobot>(
      "comms/self", rclcpp::SensorDataQoS());

    // Optional: behaviors set this robot's reported status. Defaults to STATUS_UNKNOWN.
    status_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "comms/self_status", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {status_ = msg->data;});

    last_version_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&InterRobotPublisherNode::publishSelf, this));

    RCLCPP_INFO(
      get_logger(), "Inter-robot publisher: %s->%s at %.1f Hz (version every %.1f s)",
      map_frame_.c_str(), base_frame_.c_str(), publish_rate_hz, version_period_s_);
  }

private:
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

    // Planar yaw straight from the quaternion (robot is on the ground plane).
    double yaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);

    ghost_msgs::msg::OtherRobot msg{};
    msg.x = quantizePosition(tf.transform.translation.x);
    msg.y = quantizePosition(tf.transform.translation.y);
    msg.theta = quantizeTheta(yaw);
    msg.status = status_;
    // target_x / target_y intentionally left 0 (intent is a follow-up).
    publish(msg);
  }

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
  double position_resolution_m_;
  int protocol_version_;
  double version_period_s_;

  uint8_t status_{ghost_msgs::msg::OtherRobot::STATUS_UNKNOWN};
  uint8_t seq_{0};
  rclcpp::Time last_version_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ghost_msgs::msg::OtherRobot>::SharedPtr self_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace ghost_ros_interfaces
