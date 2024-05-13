#pragma once

#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include "ghost_motion_planner_core/motion_planner.hpp"
#include "ghost_planners/robot_trajectory.hpp"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_util/angle_util.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace ghost_swerve {

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::toROSMsg;

class TrapezoidMotionPlanner : public ghost_motion_planner::MotionPlanner {
private:
	void computeTrapezoidFunction(double accel, double v_max,
	                              std::vector<double> vec_q0, std::vector<double> vec_qf);
	void computeTriangleFunction(double dist,
	                             double vel_initial, double vel_final,
	                             double accel_initial, double accel_final);
	void badCase(double vel_final, std::string error);
	std::function<double(double)> trapezoidVelocityFunc_;
	std::function<double(double)> trapezoidPositionFunc_;

	RobotTrajectory::Trajectory getTrapezoidTraj(std::vector<double> vec_q0,
	                                             std::vector<double> vec_qf,
	                                             double t0, double tf, int n);
public:
	void initialize() override;
	void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) override;
};

} // namespace ghost_swerve