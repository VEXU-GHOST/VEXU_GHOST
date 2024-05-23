/*
 *   Copyright (c) 2024 Jake Wendling
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

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

namespace ghost_swerve
{

// using ghost_planners::CubicMotionPlanner;
// using ghost_ros_interfaces::msg_helpers::toROSMsg;

class CubicMotionPlanner : public ghost_motion_planner::MotionPlanner
{
private:
  Eigen::MatrixXf computeCubicCoeff(
    double t0, double tf, std::vector<double> vec_q0,
    std::vector<double> vec_qf);
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeCubicTraj(
    std::vector<double> vec_q0,
    std::vector<double> vec_qf,
    double t0, double tf, int n);

public:
  void initialize() override;
  void generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd) override;
};

} // namespace ghost_swerve
