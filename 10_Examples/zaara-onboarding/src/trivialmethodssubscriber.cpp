#include "zaara-onboarding/trivialmethodssubscriber.hpp"

namespace myonboarding {

TrivMethodSubscriber::TrivMethodSubscriber() : Node("onboarding_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "onboarding_topic", 10, std::bind(&TrivMethodSubscriber::topic_callback, this, _1));
}

void TrivMethodSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    //adds two float values and outputs that along with message gotten from publisher
    //made two variables for the floats because I was trying to make it randomly select two values, 
    //but I couldn't figure out how I would set a range of values to choose from
    float float1 = 1.5;
    float float2 = 2.5;
    float float_addition = add_floats(float1, float2);

    RCLCPP_INFO(this->get_logger(), "Float addition value is: '%f' and message is: '%s'", float_addition, msg->data.c_str());
}

float TrivMethodSubscriber::add_floats(float a, float b) {
    return a + b;
}

}  

