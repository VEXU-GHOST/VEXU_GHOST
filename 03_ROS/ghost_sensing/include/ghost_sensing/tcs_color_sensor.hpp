#pragma once

#include <rclcpp/rclcpp.hpp>
#include <driver_tcs34725.h>
#include <driver_tcs34725_interface.h>
#include <ghost_msgs/msg/color_sensor.hpp>
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
class TCSColorSensorNode : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr m_publish_timer;
  rclcpp::Publisher<ghost_msgs::msg::ColorSensor>::SharedPtr m_color_pub;
  std::shared_ptr<color_sensor_tcs34725> m_sensor;
  int m_delay_loops = 0;

public:
  TCSColorSensorNode();

  void start();
  void timer_poll_color_sensor();

};

}
