#pragma once

#include <ghost_planners/robot_trajectory.hpp>
#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ghost_msgs/msg/drivetrain_command.hpp>
#include <ghost_msgs/msg/robot_trajectory.hpp>
#include <ghost_msgs/srv/start_recorder.hpp>
#include <ghost_msgs/srv/stop_recorder.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ghost_swerve/swerve_tree.hpp>

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
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

	// Subscribers
	void poseUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_robot_pose_sub;

	// Service Clients
	rclcpp::Client<ghost_msgs::srv::StartRecorder>::SharedPtr m_start_recorder_client;
	rclcpp::Client<ghost_msgs::srv::StopRecorder>::SharedPtr m_stop_recorder_client;

	// Swerve Model
	void updateDrivetrainMotors();
	std::shared_ptr<SwerveModel> m_swerve_model_ptr;

	// Autonomy
	std::string bt_path_;
	std::shared_ptr<SwerveTree> bt_;

	// Motion Planner
	double m_move_to_pose_kp_xy = 0.0;
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
	bool claw_auto_extended = false;

	// Tail
	bool m_tail_btn_pressed = false;
	bool m_tail_down = false;

	// Climb Mode
	bool m_climb_mode_btn_pressed = false;
	bool m_climb_mode = false;
	double lift_target;


	// Stick Mode
	bool m_tail_mode = false;

	// Bag Recorder
	bool m_recording_btn_pressed = false;
	bool m_recording = false;

	// Field vs Robot Oriented Control
	bool m_toggle_swerve_field_control_btn_pressed = false;

	// Auton Button
	double m_auton_start_time = 0.0;

	// Angle vs Velocity Control
	bool m_toggle_swerve_angle_control_btn_pressed = false;
	bool m_swerve_angle_control = false;
	double m_angle_target = 0.0;
	double m_joy_angle_control_threshold = 0.0;

	// Slew Rate Control
	double m_joystick_slew_rate = 2.0;
	double m_last_x_cmd = 0.0;
	double m_last_y_cmd = 0.0;
	double m_last_theta_cmd = 0.0;
	double m_curr_x_cmd = 0.0;
	double m_curr_y_cmd = 0.0;
	double m_curr_theta_cmd = 0.0;

	// Skills mode
	bool m_toggle_skills_control_btn_pressed = false;
	bool m_skills_control = false;
	bool m_auton_button_pressed = false;
	int m_auton_index = 0;
	bool m_teleop_started = false;

	// Burnout Prevention
	float m_burnout_current_threshold_ma;
	float m_burnout_velocity_threshold_rpm;
	long m_burnout_stall_duration_ns;
	long m_burnout_cooldown_duration_ns;

	rclcpp::Time m_intake_stall_start;
	rclcpp::Time m_intake_cooldown_start;
	bool m_intake_stalling = false;
	bool m_intake_cooling_down = false;
};

} // namespace ghost_swerve