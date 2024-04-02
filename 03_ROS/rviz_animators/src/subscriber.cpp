#include "rviz_animators/subscriber.hpp"

namespace rviz {

AnimationSubscriber::AnimationSubscriber() :
	Node("animation_subscriber"){
	// subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
	// "animation_topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, std::placeholders:: _1));
	subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, std::placeholders:: _1));
}

// void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg){
// 	RCLCPP_INFO(this->get_logger(), "Received marker message with ID %d", msg->id);
// }
void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg){
	// RCLCPP_INFO(this->get_logger(), "Received marker message with ID %d", msg->id);
	RCLCPP_INFO(this->get_logger(), "Point position: x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
}

} 