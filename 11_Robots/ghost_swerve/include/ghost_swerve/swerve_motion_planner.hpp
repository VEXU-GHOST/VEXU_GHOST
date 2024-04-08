#pragma once

#include "eigen3/Eigen/Core"
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include "ghost_motion_planner_core/motion_planner.hpp"
#include "ghost_planners/cubic_motion_planner.hpp"
#include "ghost_planners/robot_trajectory.hpp"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_util/angle_util.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace ghost_swerve {

// using ghost_planners::CubicMotionPlanner;
// using ghost_ros_interfaces::msg_helpers::toROSMsg;

class SwerveMotionPlanner : public ghost_motion_planner::MotionPlanner {
private:
	// std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
	// std::shared_ptr<SwerveModel> m_swerve_model_ptr;
	double current_x = 0.0;
	double current_y = 0.0;
	double current_x_vel = 0.0;
	double current_y_vel = 0.0;
	double current_angle = 0.0;
	double current_omega = 0.0;

	std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry> > odom_sub_;

	void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

	ghost_planners::RobotTrajectory motor_commands_from_joystick(std::vector<double> x_vel,std::vector<double> y_vel,std::vector<double> omega);

public:
	void initialize() override;
	void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) override;
};

} // namespace ghost_swerve