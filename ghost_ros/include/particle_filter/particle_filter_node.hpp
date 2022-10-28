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

#include "globals/globals.hpp"
#include "particle_filter.hpp"

using geometry::Line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::string;
using std::vector;
using Eigen::Vector2f;

using std::placeholders::_1;

namespace particle_filter{

class ParticleFilterNode : public rclcpp::Node {
  public:

  ParticleFilterNode();

  void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void PublishParticles();
  
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

};
} // namespace particle_filter