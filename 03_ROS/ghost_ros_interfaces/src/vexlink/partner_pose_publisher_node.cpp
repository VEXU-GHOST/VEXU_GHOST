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

// Reads the partner robot's pose from the VEXLink RX field of V5SensorUpdate and republishes
// it as a geometry_msgs/PoseStamped on /partner/pose. Downstream nodes (nav, viz, auton) can
// subscribe to /partner/pose without knowing about the VEXLink transport.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ghost_msgs/msg/v5_sensor_update.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

namespace ghost_ros_interfaces
{

class PartnerPosePublisherNode : public rclcpp::Node
{
public:
  PartnerPosePublisherNode()
  : rclcpp::Node("partner_pose_publisher_node")
  {
    declare_parameter("sensor_update_topic", "v5/sensor_update");
    declare_parameter("partner_pose_topic", "/partner/pose");
    declare_parameter("map_frame", "map");

    auto sensor_topic = get_parameter("sensor_update_topic").as_string();
    auto pose_topic = get_parameter("partner_pose_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();

    sensor_update_sub_ = create_subscription<ghost_msgs::msg::V5SensorUpdate>(
      sensor_topic, 10,
      std::bind(&PartnerPosePublisherNode::onSensorUpdate, this, _1));

    partner_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    RCLCPP_INFO(get_logger(), "partner_pose_publisher_node started. Listening on %s",
      sensor_topic.c_str());
  }

private:
  void onSensorUpdate(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg)
  {
    for (const auto & channel : msg->vexlink_channels) {
      if (!channel.rx_valid) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = map_frame_;

      pose_msg.pose.position.x = channel.rx_x;
      pose_msg.pose.position.y = channel.rx_y;
      pose_msg.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, channel.rx_theta);
      pose_msg.pose.orientation = tf2::toMsg(q);

      partner_pose_pub_->publish(pose_msg);
    }
  }

  std::string map_frame_;
  rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr partner_pose_pub_;
};

} // namespace ghost_ros_interfaces

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_ros_interfaces::PartnerPosePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
