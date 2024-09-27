#include "linh-onboarding/subscriber.hpp"

namespace onboarding {

Subscriber::Subscriber() :
	Node("onboarding_subsriber"){
	subscription_ = this->create_subscription<std_msgs::msg::Float32>(
		"onboarding_topic", 10, std::bind(&Subscriber::topic_callback, this, _1));

}

void Subscriber::topic_callback(const std_msgs::msg::Float32::SharedPtr msg){
	RCLCPP_INFO(this->get_logger(), "The message is: '%f'",msg->data);
}

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

int Subscriber::add(int a, int b){
    return a + b;
}
int Subscriber::subtract(int a, int b){
    return a - b;
}

void Subscriber::do_nothing(){
}


} 