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

#include "eigen3/Eigen/Geometry"

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ghost_sensing
{

class IMUFilterNode : public rclcpp::Node
{
public:
  IMUFilterNode();

private:
  void printBiasEstimates();
  void publishFilteredIMU();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  visualization_msgs::msg::MarkerArray getMarkerArrayVector3(
    const Eigen::Vector3d & vector,
    double scale);
  visualization_msgs::msg::Marker getDefaultArrowMsg(int id, builtin_interfaces::msg::Time stamp);

  // Constants
  double m_sensor_freq;

  // Bias Calibration
  int m_calibration_time;
  int m_msg_count;
  int m_num_msgs_init;
  bool m_calculate_bias = false;
  bool m_calculate_covariance = false;
  bool m_calibration_complete = false;
  std::vector<Eigen::Vector3d> m_imu_accel_samples;
  std::vector<Eigen::Vector3d> m_imu_gyro_samples;

  // Sensor Bias and Covariance
  Eigen::Vector3d m_imu_accel_bias;
  Eigen::Vector3d m_imu_gyro_bias;
  Eigen::Matrix3d m_imu_accel_bias_covariance;
  Eigen::Matrix3d m_imu_gyro_bias_covariance;
  Eigen::Matrix3d m_imu_accel_bias_covariance_base_link;
  Eigen::Matrix3d m_imu_gyro_bias_covariance_base_link;
  double m_heading_covariance;

  // Base Link Transform
  Eigen::Matrix3d m_base_link_to_sensor_rotation;
  Eigen::Vector3d m_base_link_to_sensor_translation;

  // Filtered IMU Data
  Eigen::Vector3d m_filtered_accel_vector_base_link;
  Eigen::Vector3d m_filtered_gyro_vector_base_link;

  // ROS Topics
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_filtered_imu_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_vel_viz_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_accel_viz_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_vel_filtered_viz_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_accel_filtered_viz_pub;

  sensor_msgs::msg::Imu::SharedPtr m_last_input_msg;
};

} // namespace ghost_sensing
