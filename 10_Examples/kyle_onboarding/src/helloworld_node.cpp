#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloWorldNode : public rclcpp::Node {
public:
    HelloWorldNode() : Node("helloworld_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_topic", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "hello_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
            }
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto message = std_msgs::msg::String();
                message.data = "Hello!";
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
            }
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldNode>());
    rclcpp::shutdown();
    return 0;
}
