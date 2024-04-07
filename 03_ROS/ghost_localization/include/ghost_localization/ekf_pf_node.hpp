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

	void LoadROSParams();

	// Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_sub_;

	// Publishers
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_viz_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_viz_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr world_tf_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;

	// Callback functions
	void EkfCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

	// Visualizations
	void DrawParticles(geometry_msgs::msg::PoseArray &cloud_msg);
	void PublishVisualization();
	void PublishWorldTransform();
	void DrawPredictedScan(visualization_msgs::msg::MarkerArray &viz_msg);
	void PublishMapViz();
	void PublishRobotPose(rclcpp::Time stamp);

	// Particle Filter
	particle_filter::ParticleFilter particle_filter_;
	sensor_msgs::msg::LaserScan::SharedPtr last_laser_msg_;

	// EKF
	nav_msgs::msg::Odometry last_filtered_odom_msg_;

	// Configuration
	YAML::Node config_yaml_;
	particle_filter::ParticleFilterConfig config_params;
	bool first_map_load_;
	bool laser_msg_received_;

	Eigen::Vector2f odom_loc_;
	float odom_angle_;
	geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;
	visualization_msgs::msg::MarkerArray viz_msg_;
};

}
// namespace ghost_localization