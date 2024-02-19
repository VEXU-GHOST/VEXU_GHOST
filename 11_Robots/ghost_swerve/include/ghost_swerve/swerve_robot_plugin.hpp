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
	// Publishers
	void publishVisualization();
	void publishOdometry();
	void publishTrajectoryVisualization();

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_swerve_viz_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_trajectory_viz_pub;

	// Subscribers
	void poseUpdateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_robot_pose_sub;

	// Swerve Model
	void updateDrivetrainMotors();
	std::shared_ptr<SwerveModel> m_swerve_model_ptr;

	// Autonomy
	std::string bt_path_;
	std::shared_ptr<RunTree> bt_;

	// Motion Planner
	double m_move_to_pose_kp_x = 0.0;
	double m_move_to_pose_kp_y = 0.0;
	double m_move_to_pose_kp_theta = 0.0;

	// Odometry
	Eigen::Vector2d m_last_odom_loc = Eigen::Vector2d::Zero();
	double m_last_odom_angle  = 0.0;

	Eigen::Vector2d m_curr_odom_loc = Eigen::Vector2d::Zero();
	double m_curr_odom_angle = 0.0;

	Eigen::Vector3d m_curr_odom_std = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_curr_odom_cov = Eigen::Vector3d::Zero();
	double m_k1 = 0.0;
	double m_k2 = 0.0;
	double m_k3 = 0.0;
	double m_k4 = 0.0;
	double m_k5 = 0.0;
	double m_k6 = 0.0;
	double m_k7 = 0.0;
	double m_k8 = 0.0;
	double m_k9 = 0.0;

	// Digital IO
	std::vector<bool> m_digital_io;
	std::unordered_map<std::string, size_t> m_digital_io_name_map;

	// Claw
	bool m_claw_btn_pressed = false;
	bool m_claw_open = false;

	// Tail
	bool m_tail_btn_pressed = false;
	bool m_tail_down = false;

	// Climb Mode
	bool m_climb_mode_btn_pressed = false;
	bool m_climb_mode = false;

	// Stick Mode
	bool m_tail_mode_btn_pressed = false;
	bool m_tail_mode = false;
};

} // namespace ghost_swerve