#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
  public:
    Publisher()
    : Node("onboarding_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float32();
      message.data = static_cast<float>(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}