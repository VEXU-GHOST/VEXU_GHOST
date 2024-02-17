#pragma once

#include <ghost_planners/robot_trajectory.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ghost_msgs/msg/drivetrain_command.hpp>
#include <ghost_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ghost_autonomy/run_tree.hpp>

namespace ghost_swerve {

class SwerveRobotPlugin : public ghost_ros_interfaces::V5RobotBase {
public:
	SwerveRobotPlugin();

	void initialize() override;
	void disabled() override;
	void autonomous(double current_time) override;
	void teleop(double current_time) override;
	void onNewSensorData() override;


protected:
	void publishVisualization();
	void publishOdometry();
	void publishTrajectoryVisualization();
	void poseUpdateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	void updateDrivetrainMotors();

	std::shared_ptr<SwerveModel> m_swerve_model_ptr;

	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_robot_pose_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_swerve_viz_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_trajectory_viz_pub;
	std::string bt_path_;
	std::shared_ptr<RunTree> bt_;

	// Odometry
	Eigen::Vector2d m_last_odom_loc = Eigen::Vector2d::Zero();
	double m_last_odom_angle  = 0.0;

	Eigen::Vector2d m_curr_odom_loc = Eigen::Vector2d::Zero();
	double m_curr_odom_angle = 0.0;

	Eigen::Vector3d m_curr_odom_std = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_curr_odom_cov = Eigen::Vector3d::Zero();
	double m_k1;
	double m_k2;
	double m_k3;
	double m_k4;
	double m_k5;
	double m_k6;
	double m_k7;
	double m_k8;
	double m_k9;
};

} // namespace ghost_swerve