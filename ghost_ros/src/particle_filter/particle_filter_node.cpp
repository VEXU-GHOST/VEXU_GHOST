//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "math/math_util.h"
#include "math/line2d.h"
#include "util/timer.h"

#include "particle_filter.h"

using geometry::Line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::string;
using std::vector;
using Eigen::Vector2f;

using std::placeholders::_1;

// // Create command line arguements
// DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
// DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
// DEFINE_string(init_topic,
//               "/set_pose",
//               "Name of ROS topic for initialization");

// DECLARE_int32(v);

// CONFIG_STRING(map_name_, "map");
// CONFIG_FLOAT(init_x_, "init_x");
// CONFIG_FLOAT(init_y_, "init_y");
// CONFIG_FLOAT(init_r_, "init_r");
namespace particle_filter {

class ParticleFilterNode : public rclcpp::Node {
  public:

  ParticleFilterNode(): Node("particle_filter_node"){
    // Subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "latency_test_topic",
      10,
      std::bind(&ParticleFilterNode::LaserCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "latency_test_topic",
      10,
      std::bind(&ParticleFilterNode::OdometryCallback, this, _1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "latency_test_topic",
      10,
      std::bind(&ParticleFilterNode::InitialPoseCallback, this, _1));

    // Publishers
    // publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("latency_test_topic", 10);

  }

  void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Process laser observation

    // Update robot pose estimate

    // Update unmapped obstacle scans

    // Publish Visualization
  }

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // Particle Fitler Predict

    // Update robot pose estimate

    // Publish Visualization


  }

  void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    // Set new initial pose
  }
  
  void PublishParticles(){

  }
  
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

};

void Run() {
  rclcpp::spin(std::make_shared<ParticleFilterNode>());
}
} // namespace particle_filter