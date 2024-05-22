#pragma once
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "rclcpp/rclcpp.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

class TestPublisher : public rclcpp::Node
{
private:
  // std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr_;
  rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr robot_pub_;

public:
  TestPublisher();
};
