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
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "yaml-cpp/yaml.h"

#include "ghost_estimation/particle_filter/particle_filter.hpp"

namespace ghost_localization {

class EkfPfNode : public rclcpp::Node {
public:
	EkfPfNode();

private:
	// Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;

	// Callback functions
	void EkfCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

	// Particle Filter
	particle_filter::ParticleFilter particle_filter_;
	sensor_msgs::msg::LaserScan::SharedPtr last_laser_msg_;

	// EKF
	nav_msgs::msg::Odometry last_filtered_odom_msg_;
};

} // namespace ghost_localization