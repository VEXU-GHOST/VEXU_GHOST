#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gpio_expander");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
