#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sayHello.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TalkerListener : public rclcpp::Node {
public:
  TalkerListener() : Node("talker_listener") {
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heard back: '%s'", msg->data.c_str());
      });

    timer_ = this->create_wall_timer(1s, [this]() {
      std_msgs::msg::String msg;
      msg.data = g_.greet();
      pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
    });
  }

private:
  sayHello g_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerListener>());
  rclcpp::shutdown();
  return 0;
}
