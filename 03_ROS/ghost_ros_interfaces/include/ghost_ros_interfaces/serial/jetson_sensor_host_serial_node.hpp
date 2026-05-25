/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <ghost_msgs/msg/sensor_host_device_msgs/color_sensor_state.hpp>
#include <ghost_msgs/msg/sensor_host_device_msgs/distance_sensor_state.hpp>
#include <ghost_msgs/msg/sensor_host_device_msgs/imu_state.hpp>
#include <ghost_msgs/msg/sensor_host_device_msgs/io_expander_state.hpp>
#include <ghost_serial/base_interfaces/jetson_serial_base.hpp>

#include <ghost_v5_interfaces/devices/device_config_map.hpp>

namespace ghost_ros_interfaces
{

class JetsonSensorHostSerialNode : public rclcpp::Node
{
public:
  JetsonSensorHostSerialNode();
  ~JetsonSensorHostSerialNode();

  bool initSerial();

private:
  // Process incoming/outgoing msgs w/ ROS
  void publishColorSensorUpdate(const std::vector<unsigned char> & buffer);
  void publishDistanceSensorUpdate(const std::vector<unsigned char> & buffer);
  void publishImuUpdate(const std::vector<unsigned char> & buffer);
  void publishIOExpanderUpdate(const std::vector<unsigned char> & buffer);

  // Background thread for processing serial data and maintaining serial connection
  void serialLoop();

  // Background thread to periodically check if serial data has timed out
  void serialTimeoutLoop();

  // ROS Parameters
  bool use_checksum_;
  bool verbose_;
  std::string read_msg_start_seq_;
  std::string write_msg_start_seq_;
  std::string port_name_;
  std::string backup_port_name_;

  // ROS Topics
  rclcpp::Publisher<ghost_msgs::msg::ColorSensorState>::SharedPtr color_sensor_update_pub_;
  rclcpp::Publisher<ghost_msgs::msg::DistanceSensorState>::SharedPtr distance_sensor_update_pub_;
  rclcpp::Publisher<ghost_msgs::msg::ImuState>::SharedPtr imu_update_pub_;
  rclcpp::Publisher<ghost_msgs::msg::IOExpanderState>::SharedPtr io_expander_update_pub_;

  // Serial Interface
  std::shared_ptr<ghost_serial::JetsonSerialBase> serial_base_interface_;
  std::vector<unsigned char> color_sensor_update_msg_;
  std::vector<unsigned char> distance_sensor_update_msg_;
  std::vector<unsigned char> imu_update_msg_;
  std::vector<unsigned char> io_expander_update_msg_;
  std::thread serial_thread_;
  std::thread serial_timeout_thread_;
  std::atomic_bool serial_open_;
  std::chrono::time_point<std::chrono::system_clock> last_msg_time_;
  std::mutex serial_reset_mutex_;
  bool using_backup_port_;

  // Msg Config
  int color_sensor_update_msg_len_;
  int distance_sensor_update_msg_len_;
  int imu_update_msg_len_;
  int io_expander_update_msg_len_;
};

} // namespace ghost_ros_interfaces
