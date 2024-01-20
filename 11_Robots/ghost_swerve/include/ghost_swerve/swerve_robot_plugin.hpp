#pragma once

#include <ghost_ros_interfaces/competition/v5_robot_base.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ghost_msgs/msg/drivetrain_command.hpp>
#include <ghost_msgs/msg/robot_trajectory.hpp>

#include <ghost_autonomy/run_tree.hpp>

namespace ghost_swerve {

class SwerveRobotPlugin : public ghost_ros_interfaces::V5RobotBase {
public:
	SwerveRobotPlugin();

	void initialize() override;
	void disabled() override;
	void autonomous(double current_time) override;
	void teleop(double current_time) override;

protected:
	void trajectoryCallback(const ghost_msgs::msg::RobotTrajectory::SharedPtr msg);

	rclcpp::Subscription<ghost_msgs::msg::RobotTrajectory>::SharedPtr trajectory_sub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	std::string bt_path_;
	std::shared_ptr<RunTree> bt_; 
};

} // namespace ghost_swerve