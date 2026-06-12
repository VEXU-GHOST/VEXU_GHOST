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

// Consumer side of inter-robot comms: subscribes to /comms/other_robot (the peer's state, received
// over VEXlink and republished by JetsonV5SerialNode), de-quantizes the pose, and broadcasts it as a
// map -> other_robot/base_link transform so rviz, costmaps, and tf lookups can use the peer's
// position directly. The mirror of InterRobotPublisherNode.
//
// STATUS_VERSION packets carry no pose: they set a peer-compatibility flag (target_x vs our protocol
// version) and, on mismatch, the peer's state is ignored. A fully zero packet is the signature of an
// RHI slot that has never received peer data, so it is ignored to avoid broadcasting a bogus (0, 0)
// pose; any real packet (even a pose at the field origin) advances seq and so passes the check
// regardless of status (the producer may legitimately report STATUS_UNKNOWN before a behavior sets it).

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ghost_msgs/msg/other_robot.hpp>

namespace ghost_ros_interfaces
{

class InterRobotReceiverNode : public rclcpp::Node
{
public:
  explicit InterRobotReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("inter_robot_receiver_node", options)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    child_frame_ = declare_parameter("peer_base_frame", "other_robot/base_link");
    position_resolution_m_ = declare_parameter("position_resolution_m", 0.05);
    protocol_version_ = declare_parameter("protocol_version", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<ghost_msgs::msg::OtherRobot>(
      "comms/other_robot", rclcpp::SensorDataQoS(),
      std::bind(&InterRobotReceiverNode::onOtherRobot, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Inter-robot receiver: /comms/other_robot -> TF %s->%s",
      map_frame_.c_str(), child_frame_.c_str());
  }

private:
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
    tf.child_frame_id = child_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(yaw / 2.0);
    tf.transform.rotation.w = std::cos(yaw / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }

  std::string map_frame_;
  std::string child_frame_;
  double position_resolution_m_;
  int protocol_version_;
  bool peer_compatible_{true};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<ghost_msgs::msg::OtherRobot>::SharedPtr sub_;
};

} // namespace ghost_ros_interfaces
