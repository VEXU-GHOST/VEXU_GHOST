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

#include "particle_filter/particle_filter_node.hpp"

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

  ParticleFilterNode::ParticleFilterNode(): Node("particle_filter_node"){
    // Subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10,
      std::bind(&ParticleFilterNode::LaserCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&ParticleFilterNode::OdometryCallback, this, _1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose",
      10,
      std::bind(&ParticleFilterNode::InitialPoseCallback, this, _1));

    // Publishers
    // publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("latency_test_topic", 10);

  }

  void ParticleFilterNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Process laser observation

    // Update robot pose estimate

    // Update unmapped obstacle scans

    // Publish Visualization
    std::cout << "Laser" << std::endl;
    std::cout << globals::test << std::endl;
  }

  void ParticleFilterNode::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // Particle Filter Predict

    // Update robot pose estimate

    // Publish Visualization


  }

  void ParticleFilterNode::InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    // Set new initial pose
    std::cout << "New Initial Pose" << std::endl;
    globals::test = "test20";
  }
  
  void ParticleFilterNode::PublishParticles(){

  }
  
} // namespace particle_filter