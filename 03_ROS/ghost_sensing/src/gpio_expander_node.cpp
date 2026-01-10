#include "rclcpp/rclcpp.hpp"
#include "TCA9536.h"   // adjust include to whatever your header is called

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gpio_expander");

  // parameters (adjust names to match your team’s style)
  node->declare_parameter<int>("address", 0x41);        // example
  node->declare_parameter<std::string>("i2c_device", "/dev/i2c-1");

  const int address = node->get_parameter("address").as_int();
  const auto dev = node->get_parameter("i2c_device").as_string();
  RCLCPP_INFO(node->get_logger(), "GPIO expander starting: address=0x%X dev=%s",
              address, dev.c_str());

  // TODO: construct I2C + TCA9536 objects here, then provide services/topics

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}