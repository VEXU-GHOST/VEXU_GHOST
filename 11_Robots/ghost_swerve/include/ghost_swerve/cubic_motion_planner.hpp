#pragma once

#include "eigen3/Eigen/Core"
#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include "ghost_motion_planner_core/motion_planner.hpp"
#include "ghost_planners/robot_trajectory.hpp"
#include "ghost_swerve/swerve_model.hpp"
#include "ghost_util/angle_util.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace ghost_swerve {

// using ghost_planners::CubicMotionPlanner;
// using ghost_ros_interfaces::msg_helpers::toROSMsg;

class CubicMotionPlanner : public ghost_motion_planner::MotionPlanner {
private:
	Eigen::MatrixXf computeCubicCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf);
	std::tuple<std::vector<double>, std::vector<double>, std::vector<double> > computeCubicTraj(std::vector<double> vec_q0,
	                                                                                            std::vector<double> vec_qf,
	                                                                                            double t0, double tf, int n);
public:
	void initialize() override;
	void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) override;
};

} // namespace ghost_swerve