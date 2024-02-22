// ========================================================================
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
// ========================================================================
/*!
   \file    particle-filter-main.cc
   \brief   Main entry point for particle filter based
         mobile robot localization
   \author  Joydeep Biswas, (C) 2019
 */
// ========================================================================

#include "eigen3/Eigen/Geometry"
#include "ghost_localization/pf_ekf_node.hpp"
#include "ghost_util/angle_util.hpp"
#include "math/line2d.h"
#include "math/math_util.h"
#include "util/timer.h"

using Eigen::Vector2f;
using geometry::Line;
using geometry::Line2f;
using math_util::DegToRad;
using math_util::RadToDeg;
using particle_filter::ParticleFilter;
using particle_filter::ParticleFilterConfig;
using std::placeholders::_1;
using std::string;
using std::vector;

namespace ghost_localization {

PfEkfNode::PfEkfNode() :
	Node("pf_ekf_node"){
	// Subscribers
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/sensors/wheel_odom",
		10,
		std::bind(&PfEkfNode::OdomCallback, this, _1));

	laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"/scan",
		rclcpp::SensorDataQoS(),
		std::bind(&PfEkfNode::LaserCallback, this, _1));

	auto map_qos = rclcpp::QoS(10);
	map_qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

	set_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/initial_pose",
		10,
		std::bind(&PfEkfNode::InitialPoseCallback, this, _1)
		);

	// Publishers
	debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("estimation_debug", 10);
	cloud_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);
	map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_viz", map_qos);
	// world_tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
	ekf_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pf_odometry", 10);

	// Use simulated time in ROS TODO:!!!!
	rclcpp::Parameter use_sim_time_param("use_sim_time", true);
	this->set_parameter(use_sim_time_param);

	LoadROSParams();

	particle_filter_ = ParticleFilter(config_params);
	first_map_load_ = true;
	laser_msg_received_ = false;

	const Vector2f init_loc(config_params.init_x, config_params.init_y);
	const float init_angle = config_params.init_r;

	odom_loc_(config_params.init_x, config_params.init_y);
	odom_angle_ = config_params.init_r;

	particle_filter_.Initialize(config_params.map, init_loc, config_params.init_r);
}

void PfEkfNode::LoadROSParams(){
	config_params = ParticleFilterConfig();

	declare_parameter("particle_filter.world_frame", "");
	config_params.world_frame = get_parameter("particle_filter.world_frame").as_string();

	declare_parameter("particle_filter.map", "");
	config_params.map = get_parameter("particle_filter.map").as_string();
	RCLCPP_INFO (
		this->get_logger(),
		"Initialize map: %s",
		config_params.map.c_str());

	declare_parameter("particle_filter.init_x", 0.0);
	declare_parameter("particle_filter.init_y", 0.0);
	declare_parameter("particle_filter.init_r", 0.0);
	config_params.init_x = get_parameter("particle_filter.init_x").as_double();
	config_params.init_y = get_parameter("particle_filter.init_y").as_double();
	config_params.init_r = get_parameter("particle_filter.init_r").as_double();

	declare_parameter("particle_filter.resample_frequency", 1);
	config_params.resample_frequency = get_parameter("particle_filter.resample_frequency").as_int();

	declare_parameter("particle_filter.init_x_sigma", 0.0);
	declare_parameter("particle_filter.init_y_sigma", 0.0);
	declare_parameter("particle_filter.init_r_sigma", 0.0);
	config_params.init_x_sigma = get_parameter("particle_filter.init_x_sigma").as_double();
	config_params.init_y_sigma = get_parameter("particle_filter.init_y_sigma").as_double();
	config_params.init_r_sigma = get_parameter("particle_filter.init_r_sigma").as_double();

	declare_parameter("particle_filter.k1", 0.0);
	declare_parameter("particle_filter.k2", 0.0);
	declare_parameter("particle_filter.k3", 0.0);
	declare_parameter("particle_filter.k4", 0.0);
	declare_parameter("particle_filter.k5", 0.0);
	declare_parameter("particle_filter.k6", 0.0);
	declare_parameter("particle_filter.k7", 0.0);
	declare_parameter("particle_filter.k8", 0.0);
	declare_parameter("particle_filter.k9", 0.0);
	config_params.k1 = get_parameter("particle_filter.k1").as_double();
	config_params.k2 = get_parameter("particle_filter.k2").as_double();
	config_params.k3 = get_parameter("particle_filter.k3").as_double();
	config_params.k4 = get_parameter("particle_filter.k4").as_double();
	config_params.k5 = get_parameter("particle_filter.k5").as_double();
	config_params.k6 = get_parameter("particle_filter.k6").as_double();
	config_params.k7 = get_parameter("particle_filter.k7").as_double();
	config_params.k8 = get_parameter("particle_filter.k8").as_double();
	config_params.k9 = get_parameter("particle_filter.k9").as_double();

	declare_parameter("particle_filter.laser_offset", 0.0);
	declare_parameter("particle_filter.laser_angle_offset", 0.0);
	declare_parameter("particle_filter.min_update_dist", 0.0);
	declare_parameter("particle_filter.min_update_angle", 0.0);
	config_params.laser_offset = get_parameter("particle_filter.laser_offset").as_double();
	config_params.laser_angle_offset = get_parameter("particle_filter.laser_angle_offset").as_double();
	config_params.min_update_dist = get_parameter("particle_filter.min_update_dist").as_double();
	config_params.min_update_angle = get_parameter("particle_filter.min_update_angle").as_double();

	declare_parameter("particle_filter.sigma_observation", 0.0);
	declare_parameter("particle_filter.gamma", 0.0);
	declare_parameter("particle_filter.dist_short", 0.0);
	declare_parameter("particle_filter.dist_long", 0.0);
	declare_parameter("particle_filter.range_min", 0.0);
	declare_parameter("particle_filter.range_max", 0.0);
	declare_parameter("particle_filter.resize_factor", 0.0);
	declare_parameter("particle_filter.num_particles", 50);
	config_params.sigma_observation = get_parameter("particle_filter.sigma_observation").as_double();
	config_params.gamma = get_parameter("particle_filter.gamma").as_double();
	config_params.dist_short = get_parameter("particle_filter.dist_short").as_double();
	config_params.dist_long = get_parameter("particle_filter.dist_long").as_double();
	config_params.range_min = get_parameter("particle_filter.range_min").as_double();
	config_params.range_max = get_parameter("particle_filter.range_max").as_double();
	config_params.resize_factor = get_parameter("particle_filter.resize_factor").as_double();
	config_params.num_particles = get_parameter("particle_filter.num_particles").as_int();

	declare_parameter("particle_filter.use_skip_range", false);
	declare_parameter("particle_filter.skip_index_min", 0);
	declare_parameter("particle_filter.skip_index_max", 0);

	config_params.use_skip_range = get_parameter("particle_filter.use_skip_range").as_bool();
	config_params.skip_index_min = get_parameter("particle_filter.skip_index_min").as_int();
	config_params.skip_index_max = get_parameter("particle_filter.skip_index_max").as_int();
}

void PfEkfNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
	if(!laser_msg_received_){
		laser_msg_received_ = true;
	}
	try{
		last_laser_msg_ = msg;
		particle_filter_.ObserveLaser(
			msg->ranges,
			msg->range_min,
			msg->range_max,
			msg->angle_min + config_params.laser_angle_offset,
			msg->angle_max + config_params.laser_angle_offset);

		PublishRobotPose();
		// PublishWorldTransform();
		PublishVisualization();
	}
	catch(std::exception e){
		RCLCPP_ERROR(this->get_logger(), "Laser : % s ", e.what());
	}
}


void PfEkfNode::InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
	try{
		// Set new initial pose
		const Vector2f init_loc(msg->pose.pose.position.x, msg->pose.pose.position.y);
		const float init_angle = 2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

		RCLCPP_INFO(
			this->get_logger(),
			"Initialize : % s (% f,% f) % f\u00b0 \n ",
			config_params.map.c_str(),
			init_loc.x(),
			init_loc.y(),
			RadToDeg(init_angle));

		particle_filter_.Initialize(config_params.map, init_loc, init_angle);

		// PublishWorldTransform();
		PublishVisualization();
		PublishMapViz();
	}
	catch(std::exception e){
		RCLCPP_ERROR(this->get_logger(), "Initial Pose:% s ", e.what());
	}
}

// Odometry
void PfEkfNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
	this->last_filtered_odom_msg_ = *msg;
	odom_loc_(last_filtered_odom_msg_.pose.pose.position.x, last_filtered_odom_msg_.pose.pose.position.y);
	odom_angle_ = 2.0 * atan2(last_filtered_odom_msg_.pose.pose.orientation.z, last_filtered_odom_msg_.pose.pose.orientation.w);
	try{
		Vector2f robot_loc(0, 0);
		float robot_angle(0);
		std::array<double,36> covariance;
		particle_filter_.Predict(odom_loc_, odom_angle_);
		particle_filter_.GetLocation(&robot_loc, &robot_angle, &covariance);

		PublishRobotPose();
		// PublishWorldTransform();
		PublishVisualization();
	}
	catch(std::exception e){
		RCLCPP_ERROR(this->get_logger(), "Odom: %s", e.what());
	}
}

void PfEkfNode::PublishRobotPose(){
	Vector2f robot_loc(0, 0);
	float robot_angle(0);
	std::array<double, 36> covariance;
	particle_filter_.GetLocation(&robot_loc, &robot_angle, &covariance);

	auto robot_pose_ = geometry_msgs::msg::PoseWithCovarianceStamped{};

	robot_pose_.header.stamp = this->get_clock()->now();
	robot_pose_.header.frame_id = "base_link";

	robot_pose_.pose.pose.position.x = robot_loc.x();
	robot_pose_.pose.pose.position.y = robot_loc.y();
	robot_pose_.pose.pose.position.z = 0.0; // TODO ??
	robot_pose_.pose.covariance = covariance;
	ekf_odom_pub_->publish(robot_pose_);
}

void PfEkfNode::PublishMapViz(){
	auto map_msg = visualization_msgs::msg::Marker{};
	auto map = particle_filter_.GetMap();

	// Iterate through all lines in map
	for(size_t i = 0; i < map.lines.size(); ++i){
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
	map_msg.header.frame_id = config_params.world_frame;
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

void PfEkfNode::PublishWorldTransform(){
	auto tf_msg = tf2_msgs::msg::TFMessage{};
	auto world_to_base_tf = geometry_msgs::msg::TransformStamped{};
	world_to_base_tf.header.stamp = this->get_clock()->now();
	world_to_base_tf.header.frame_id = config_params.world_frame;
	world_to_base_tf.child_frame_id = "base_link";

	Vector2f robot_loc(0, 0);
	float robot_angle(0);
	std::array<double, 36> covariance;
	particle_filter_.GetLocation(&robot_loc, &robot_angle, &covariance);

	world_to_base_tf.transform.translation.x = robot_loc.x();
	world_to_base_tf.transform.translation.y = robot_loc.y();
	world_to_base_tf.transform.translation.z = 0.0;
	world_to_base_tf.transform.rotation.x = 0.0;
	world_to_base_tf.transform.rotation.y = 0.0;
	world_to_base_tf.transform.rotation.z = sin(robot_angle * 0.5);
	world_to_base_tf.transform.rotation.w = cos(robot_angle * 0.5);

	tf_msg.transforms.push_back(world_to_base_tf);
	// world_tf_pub_->publish(tf_msg);
}

void PfEkfNode::PublishVisualization(){
	// static double t_last = 0;

	// // if (GetMonotonicTime() - t_last < 1/30.0)
	// // {
	// //   // Rate-limit visualization.
	// //   return;
	// // }
	// t_last = GetMonotonicTime();
	// Publish Particle Cloud
	auto cloud_msg = geometry_msgs::msg::PoseArray{};
	cloud_msg.header.frame_id = config_params.world_frame;
	cloud_msg.header.stamp = this->get_clock()->now();
	DrawParticles(cloud_msg);
	cloud_viz_pub_->publish(cloud_msg);

	// Publish Debug Markers
	viz_msg_ = visualization_msgs::msg::MarkerArray{};
	DrawPredictedScan(viz_msg_);

	if(first_map_load_){
		PublishMapViz();
		first_map_load_ = false;
	}
	debug_viz_pub_->publish(viz_msg_);
}

void PfEkfNode::DrawParticles(geometry_msgs::msg::PoseArray &cloud_msg){
	vector<particle_filter::Particle> particles;
	particle_filter_.GetParticles(&particles);
	for(const particle_filter::Particle &p : particles){
		auto pose_msg = geometry_msgs::msg::Pose{};
		pose_msg.position.x = p.loc.x();
		pose_msg.position.y = p.loc.y();
		pose_msg.orientation.w = cos(p.angle * 0.5);
		pose_msg.orientation.z = sin(p.angle * 0.5);
		cloud_msg.poses.push_back(pose_msg);
	}
}

void PfEkfNode::DrawPredictedScan(visualization_msgs::msg::MarkerArray &viz_msg) {
	if(!laser_msg_received_){
		return;
	}
	Vector2f robot_loc(0, 0);
	float robot_angle(0);
	std::array<double, 36> covariance;
	particle_filter_.GetLocation(&robot_loc, &robot_angle, &covariance);
	vector<Vector2f> predicted_scan;

	particle_filter_.GetPredictedPointCloud(
		robot_loc,
		robot_angle,
		last_laser_msg_->ranges.size(),
		last_laser_msg_->range_min,
		last_laser_msg_->range_max,
		last_laser_msg_->angle_min + config_params.laser_angle_offset,
		last_laser_msg_->angle_max + config_params.laser_angle_offset,
		&predicted_scan);

	auto predicted_scan_msg = visualization_msgs::msg::Marker{};
	predicted_scan_msg.header.stamp = this->get_clock()->now();
	predicted_scan_msg.header.frame_id = config_params.world_frame;
	predicted_scan_msg.id = 1;
	predicted_scan_msg.type = 8; // Points
	predicted_scan_msg.color.b = 1.0;
	predicted_scan_msg.color.a = 1.0;
	predicted_scan_msg.scale.x = 0.04;
	predicted_scan_msg.scale.y = 0.04;

	for(std::size_t i = 0; i < predicted_scan.size(); i++){
		int laser_index = i * config_params.resize_factor;
		if(!config_params.use_skip_range || (laser_index < config_params.skip_index_min) || (laser_index > config_params.skip_index_max) ){
			// Transform particle to map
			auto point_msg = geometry_msgs::msg::Point{};
			point_msg.x = predicted_scan[i].x();
			point_msg.y = predicted_scan[i].y();
			predicted_scan_msg.points.push_back(point_msg);
		}
	}
	viz_msg.markers.push_back(predicted_scan_msg);

	// Publish observed scan in world frame
	auto true_scan_msg = visualization_msgs::msg::Marker{};
	true_scan_msg.header.stamp = this->get_clock()->now();
	true_scan_msg.header.frame_id = config_params.world_frame;
	true_scan_msg.id = 2;
	true_scan_msg.type = 8; // Points
	true_scan_msg.color.r = 1.0;
	true_scan_msg.color.a = 1.0;
	true_scan_msg.scale.x = 0.02;
	true_scan_msg.scale.y = 0.02;

	auto rot_bl_to_world = Eigen::Rotation2D<float>(robot_angle).toRotationMatrix();
	for(std::size_t i = 0; i < last_laser_msg_->ranges.size(); i++){
		int laser_index = ((int) (i / config_params.resize_factor)) * config_params.resize_factor;
		if(!config_params.use_skip_range || (laser_index < config_params.skip_index_min) || (laser_index > config_params.skip_index_max) ){
			// Transform particle to map
			float range = last_laser_msg_->ranges[i];
			if((range >= config_params.range_min) && (range <= config_params.range_max) ){
				float angle = last_laser_msg_->angle_min + i * last_laser_msg_->angle_increment + config_params.laser_angle_offset + robot_angle;

				Eigen::Vector2f p = Eigen::Vector2f(range * cos(angle), range * sin(angle)) + robot_loc + rot_bl_to_world * Eigen::Vector2f(config_params.laser_offset, 0.0);
				auto point_msg = geometry_msgs::msg::Point{};
				point_msg.x = p.x();
				point_msg.y = p.y();
				true_scan_msg.points.push_back(point_msg);
			}
		}
	}
	viz_msg.markers.push_back(true_scan_msg);
}

} // namespace ghost_localization

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_localization::PfEkfNode>());
	rclcpp::shutdown();
	return 0;
}