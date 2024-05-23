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

#include <vector>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "yaml-cpp/yaml.h"

#include "ghost_estimation/particle_filter/particle_filter.hpp"
#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

namespace ghost_ros
{

class GhostEstimatorNode : public rclcpp::Node
{
public:
  GhostEstimatorNode();

private:
  void LoadROSParams();

  // Topic callback functions
  void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void EncoderCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
  void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Topic publish functions
  void PublishWorldTransform();
  void PublishVisualization();
  void PublishMapViz();
  void PublishJointStateMsg(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
  void PublishGhostRobotState(const ghost_msgs::msg::V5SensorUpdate::SharedPtr sensor_update_msg);


  // Visualization
  void DrawWheelAxisVectors(std::vector<geometry::Line2f> & lines);
  void DrawICRPoints(std::vector<Eigen::Vector3f> & points);
  void DrawParticles(geometry_msgs::msg::PoseArray & cloud_msg);
  void DrawPredictedScan(visualization_msgs::msg::MarkerArray & viz_msg);

  visualization_msgs::msg::MarkerArray viz_msg_;

  void CalculateHSpaceICR(ghost_msgs::msg::V5SensorUpdate::SharedPtr encoder_msg);
  void CalculateOdometry(ghost_msgs::msg::V5SensorUpdate::SharedPtr encoder_msg);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr encoder_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr world_tf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<ghost_msgs::msg::GhostRobotState>::SharedPtr robot_state_pub_;


  // Particle Filter
  particle_filter::ParticleFilter particle_filter_;
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_msg_;

  // Configuration
  YAML::Node config_yaml_;
  particle_filter::ParticleFilterConfig config_params;
  bool first_map_load_;
  bool laser_msg_received_;

  // Odometry Config
  Eigen::Vector2f left_wheel_link_;
  Eigen::Vector2f right_wheel_link_;
  Eigen::Vector2f back_wheel_link_;

  Eigen::Vector3f h_space_icr_avg_;
  Eigen::Vector3f icr_flat_estimation_;

  Eigen::Vector2f odom_loc_;
  float odom_angle_;

  Eigen::MatrixXf A_;
  Eigen::MatrixXf A_pinv_;
  float x_vel_;
  float y_vel_;
  float theta_vel_;
};

} // namespace ghost_ros
