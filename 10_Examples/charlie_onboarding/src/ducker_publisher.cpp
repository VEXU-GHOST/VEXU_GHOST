#include "charlie_onboarding/ducker_publisher.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher(const rclcpp::NodeOptions & options)
: Node("minimal_publisher", options)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

void MinimalPublisher::timer_callback()
{
  std_msgs::msg::String message;
  message.data = amogus_.next();
  RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

#ifdef DUCKER_STANDALONE
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
#endif
