#include "pub_sub.cpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  // This line is always required if you are going to instantiate a ROS node.
  // For testing regular C++ libraries, it should be excluded.

  // Run this if you don't want to run the tests
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<will_onboarding::PubNode>());
  rclcpp::spin(std::make_shared<will_onboarding::SubNode>());
  rclcpp::shutdown();
  return 0;

}
