#include "Johnny_Shen_onboarding/a.hpp"

Publishertest::Publishertest()
: rclcpp::Node("hello_node")
{
  robot_pub_ = create_publisher<std_msgs::msg::String>(
    "/hello",
    10);
  std_msgs::msg::String hello_msg{};
  hello_msg.data = "hello";
  robot_pub_->publish(hello_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Publishertest>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}