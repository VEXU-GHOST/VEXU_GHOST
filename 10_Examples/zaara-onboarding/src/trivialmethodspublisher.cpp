#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"


//a little confused about what chrono_literals is
using namespace std::chrono_literals;

class TrivMethodPublisher : public rclcpp::Node
{
  public:
    TrivMethodPublisher()
    : Node("onboarding_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("onboarding_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&TrivMethodPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      //changing counter to be a string 
      message.data = "Useless Counter: " + std::to_string(count_+=2);
    
      RCLCPP_INFO(this->get_logger(), "'%s'", message.data.c_str());
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrivMethodPublisher>());
  rclcpp::shutdown();
  return 0;
}