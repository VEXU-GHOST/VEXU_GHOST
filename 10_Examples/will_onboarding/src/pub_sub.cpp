#include "will_onboarding/pub_sub.hpp"

using namespace will_onboarding;

PubNode::PubNode() : Node("pub_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("greeting", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PubNode::timer_callback, this)
    );
    this->declare_parameter<bool>("say_hi", true);
}

std::string PubNode::create_hi_msg() {
    return "hi";
}

std::string PubNode::create_bye_msg() {
    return "bye";
}

void PubNode::timer_callback() {
    bool say_hi = this->get_parameter("say_hi").get_parameter_value().get<bool>();
    std_msgs::msg::String msg;
    msg.data = say_hi ? create_hi_msg() : create_bye_msg();
    publisher_->publish(msg);
}

SubNode::SubNode() : Node("sub_node") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "greeting",
        10,
        std::bind(&SubNode::topic_callback, this, std::placeholders::_1)
    );
}

void SubNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << "Received: '" << msg->data << "'" << std::endl;
}
