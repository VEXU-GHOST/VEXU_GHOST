#pragma once

#include <rclcpp/rclcpp.hpp>
#include <driver_apds9960.h>
#include <driver_tcs34725_interface.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <functional>


namespace ghost_sensing
{
class AvagoColorSensorNode : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr m_publish_timer;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr m_rgb_pub;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr m_hsv_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_prox_pub;
  std::shared_ptr<color_sensor_apds9960> m_sensor;
  int m_delay_loops = 0;

public:
  AvagoColorSensorNode();

  void timer_poll_color_sensor();
  int init();

  double m_poll_freq;
};

}
