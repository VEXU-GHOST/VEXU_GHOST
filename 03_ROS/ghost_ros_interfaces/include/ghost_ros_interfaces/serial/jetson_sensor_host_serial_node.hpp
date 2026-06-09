/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.
 *
 *   MIT License (see repository).
 */

#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <ghost_msgs/msg/color_sensor_state.hpp>

#include "ghost_ros_interfaces/sensor_host/protocol.hpp"

namespace ghost_ros_interfaces
{

// Drives the sensor host (a generic I2C bridge) over USB serial: configures the
// ISL29125 colour sensor, requests a recurring read of its colour registers,
// and publishes each decoded reading. See ghost_sensor_host/PROTOCOL.md.
class JetsonSensorHostSerialNode : public rclcpp::Node
{
public:
  JetsonSensorHostSerialNode();
  ~JetsonSensorHostSerialNode();

private:
  bool openSerial();
  void writeFrame(const std::vector<uint8_t> & wire);
  void sendColorInit();
  void sendColorReadRequest();
  void readLoop();
  void handleReadResult(const sensor_host::ReadResult & result);

  // Parameters
  std::string serial_port_;
  int color_port_;     // protocol port (input − 1); input 7 -> 6
  int color_addr_;     // I2C address (rotary DAC dependent)
  int read_count_;
  int interval_ms_;
  uint16_t color_read_id_;

  int fd_;
  std::atomic_bool running_;
  std::thread read_thread_;
  sensor_host::FrameAccumulator accumulator_;

  rclcpp::Publisher<ghost_msgs::msg::ColorSensorState>::SharedPtr color_pub_;
  rclcpp::TimerBase::SharedPtr rearm_timer_;
};

}  // namespace ghost_ros_interfaces
