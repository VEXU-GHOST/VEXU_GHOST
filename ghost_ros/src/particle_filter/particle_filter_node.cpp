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

using geometry::Line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::string;
using std::vector;
using Eigen::Vector2f;
using std::placeholders::_1;
namespace particle_filter {

  ParticleFilterNode::ParticleFilterNode(std::string config_file): Node("particle_filter_node"){
    // Use simulated time in ROS
    rclcpp::Parameter use_sim_time_param("use_sim_time", true);
    this->set_parameter(use_sim_time_param);
    
    // Subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10,
      std::bind(&ParticleFilterNode::LaserCallback, this, _1));

    odom_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "odom",
      10,
      std::bind(&ParticleFilterNode::OdometryCallback, this, _1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose",
      10,
      std::bind(&ParticleFilterNode::InitialPoseCallback, this, _1));

    // Publishers
    cloud_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);
    map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_viz", 10);
    world_tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

    // Loads configuration from YAML
    LoadConfiguration(config_file);

    particle_filter_ = ParticleFilter(config_params);    
    last_laser_msg_ = sensor_msgs::msg::LaserScan{};
    first_map_load_ = true;

    const Vector2f init_loc(config_params.init_x, config_params.init_y);
    const float init_angle = config_params.init_r;
    particle_filter_.Initialize(config_params.map, init_loc, config_params.init_r);
  }

  void ParticleFilterNode::LoadConfiguration(std::string filename){
    config_yaml = YAML::LoadFile(filename);

    config_params = ParticleFilterConfig();

    config_params.map = globals::repo_base_dir + config_yaml["map"].as<std::string>();
    config_params.init_x = config_yaml["init_x"].as<float>();
    config_params.init_y = config_yaml["init_y"].as<float>();
    config_params.init_r = config_yaml["init_r"].as<float>();
    config_params.num_particles = config_yaml["num_particles"].as<int>();
    config_params.resample_frequency = config_yaml["resample_frequency"].as<int>();
    config_params.init_x_sigma = config_yaml["init_x_sigma"].as<float>();
    config_params.init_y_sigma = config_yaml["init_y_sigma"].as<float>();
    config_params.init_r_sigma = config_yaml["init_r_sigma"].as<float>();
    config_params.k1 = config_yaml["k1"].as<float>();
    config_params.k2 = config_yaml["k2"].as<float>();
    config_params.k3 = config_yaml["k3"].as<float>();
    config_params.k4 = config_yaml["k4"].as<float>();
    config_params.k5 = config_yaml["k5"].as<float>();
    config_params.k6 = config_yaml["k6"].as<float>();
    config_params.laser_offset = config_yaml["laser_offset"].as<float>();
    config_params.min_update_dist = config_yaml["min_update_dist"].as<float>();
    config_params.min_update_angle = config_yaml["min_update_angle"].as<float>();
    config_params.sigma_observation = config_yaml["sigma_observation"].as<double>();
    config_params.gamma = config_yaml["gamma"].as<double>();
    config_params.dist_short = config_yaml["dist_short"].as<double>();
    config_params.dist_long = config_yaml["dist_long"].as<double>();
    config_params.range_min = config_yaml["range_min"].as<double>();
    config_params.range_max = config_yaml["range_max"].as<double>();
    config_params.resize_factor = config_yaml["resize_factor"].as<double>();
  }

  void ParticleFilterNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Process laser observation

    // Update robot pose estimate

    // Update unmapped obstacle scans

    PublishWorldTransform();
    PublishVisualization();
  }

  void ParticleFilterNode::OdometryCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
    // Particle Filter Predict

    // Update robot pose estimate

    PublishWorldTransform();
    PublishVisualization();
  }

  void ParticleFilterNode::InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    // Set new initial pose
    const Vector2f init_loc(msg->pose.pose.position.x, msg->pose.pose.position.y);
    const float init_angle = 2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    
    RCLCPP_INFO(
      this->get_logger(),
      "Initialize: %s (%f,%f) %f\u00b0\n",
      config_params.map.c_str(),
      init_loc.x(),
      init_loc.y(),
      RadToDeg(init_angle));
      
    particle_filter_.Initialize(config_params.map, init_loc, init_angle);

    PublishWorldTransform();
    PublishVisualization();
    PublishMapViz();
  }
  
  void ParticleFilterNode::DrawParticles(geometry_msgs::msg::PoseArray &viz_msg){
    vector<particle_filter::Particle> particles;
    particle_filter_.GetParticles(&particles);
    for (const particle_filter::Particle& p : particles) {
      auto pose_msg = geometry_msgs::msg::Pose{};
      pose_msg.position.x = p.loc.x();
      pose_msg.position.y = p.loc.y();
      pose_msg.orientation.w = cos(p.angle * 0.5);
      pose_msg.orientation.z = sin(p.angle * 0.5);
      viz_msg.poses.push_back(pose_msg);
    }
  }

  void ParticleFilterNode::PublishWorldTransform(){
    auto tf_msg = tf2_msgs::msg::TFMessage{};

    auto world_to_base_tf = geometry_msgs::msg::TransformStamped{};
    world_to_base_tf.header.stamp = this->get_clock()->now();
    world_to_base_tf.header.frame_id = "world";
    world_to_base_tf.child_frame_id = "base_link";
    
    Vector2f robot_loc(0, 0);
    float robot_angle(0);
    particle_filter_.GetLocation(&robot_loc, &robot_angle);

    world_to_base_tf.transform.translation.x = robot_loc.x();
    world_to_base_tf.transform.translation.y = robot_loc.y();
    world_to_base_tf.transform.translation.z = 0.0;
    world_to_base_tf.transform.rotation.x = 0.0;
    world_to_base_tf.transform.rotation.y = 0.0;
    world_to_base_tf.transform.rotation.z = sin(robot_angle * 0.5);
    world_to_base_tf.transform.rotation.w = cos(robot_angle * 0.5);

    tf_msg.transforms.push_back(world_to_base_tf);
    world_tf_pub_->publish(tf_msg);
  }

  void ParticleFilterNode::PublishMapViz(){
    auto map_msg = visualization_msgs::msg::Marker{};
    auto map = particle_filter_.GetMap();

    // Iterate through all lines in map
    for (size_t i = 0; i < map.lines.size(); ++i) {
      const geometry::Line2f line = map.lines[i];

      auto start_point = geometry_msgs::msg::Point{};
      start_point.x = line.p0.x();
      start_point.y = line.p0.y();

      auto end_point = geometry_msgs::msg::Point{};
      end_point.x = line.p1.x();
      end_point.y = line.p1.y();

      map_msg.points.push_back(start_point);
      map_msg.points.push_back(end_point);
      }

  map_msg.header.stamp = this->get_clock()->now();
  map_msg.header.frame_id = "world";
  map_msg.id = 0;
  map_msg.type = 5;   // Line List
  map_msg.action = 0; // Add / Modify
  map_msg.scale.x = 0.01;
  map_msg.color.r = 0.0;
  map_msg.color.g = 0.0;
  map_msg.color.b = 0.0;
  map_msg.color.a = 1.0;

  map_viz_pub_->publish(map_msg);
  }

  void ParticleFilterNode::PublishVisualization(){
    static double t_last = 0;
    if (GetMonotonicTime() - t_last < 0.05) {
      // Rate-limit visualization.
      return;
    }
    t_last = GetMonotonicTime();

    // Publish Particle Cloud
    auto viz_msg = geometry_msgs::msg::PoseArray{};
    viz_msg.header.frame_id = "world";
    viz_msg.header.stamp = this->get_clock()->now();
    DrawParticles(viz_msg);
    cloud_viz_pub_->publish(viz_msg);

    if(first_map_load_){
      PublishMapViz();
      first_map_load_ = false;
    }
  }
  
} // namespace particle_filter