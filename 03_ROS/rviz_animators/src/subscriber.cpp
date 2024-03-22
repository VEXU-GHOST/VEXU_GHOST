#include "rviz_animators/subscriber.hpp"

namespace rviz_animators {

AnimationSubscriber::AnimationSubscriber() :
	Node("animation_subscriber"){
	subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
		"animation_topic", 10, std::bind(&AnimationSubscriber::topic_callback, this, _1));
}

void AnimationSubscriber::topic_callback(visualization_msgs::msg::Marker::SharedPtr msg){
	RCLCPP_INFO(this->get_logger(), "Received marker message with ID %d", msg->id);
}

rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;

} 