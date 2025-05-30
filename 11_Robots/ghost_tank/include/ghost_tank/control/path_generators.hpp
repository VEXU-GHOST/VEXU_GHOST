/*
 *   Copyright (c) 2024 Maxx Wilson
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

#include <ghost_tank/control/trajectory.hpp>
#include <eigen3/Eigen/Core>

namespace ghost_tank
{

namespace motion_planning
{

enum class trajectory_type_e
{
  CUBIC_BEZIER,
  QUADRATIC_BEZIER
};

const std::unordered_map<std::string, trajectory_type_e> TRAJECTORY_STRING_ENUM_MAP{
  {"CUBIC_BEZIER", trajectory_type_e::CUBIC_BEZIER},
  {"QUADRATIC_BEZIER", trajectory_type_e::QUADRATIC_BEZIER}
};

Trajectory generateCubicBezierCurve(
  Eigen::Vector2d start_point,
  double start_angle_rad,
  Eigen::Vector2d end_point,
  double end_angle_rad,
  double lead,
  int num_points = 250)
{
  Trajectory path(num_points);

  const Eigen::Vector2d & P0 = start_point;
  const Eigen::Vector2d & P1 = start_point + lead * Eigen::Vector2d(cos(start_angle_rad), sin(start_angle_rad));
  const Eigen::Vector2d & P2 = end_point - lead * Eigen::Vector2d(cos(end_angle_rad), sin(end_angle_rad));
  const Eigen::Vector2d & P3 = end_point;

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / static_cast<double>(num_points);
    Eigen::Vector2d p = pow(1 - t, 3) * P0 + 3 * pow(1 - t, 2) * t * P1 + 3 * (1 - t) * pow(t, 2) * P2 + pow(t, 3) * P3;
    Eigen::Vector2d v = 3 * pow(1 - t, 2) * (P1 - P0) + 6 * (1 - t) * t * (P2 - P1) + 3 * pow(t, 2) * (P3 - P2);
    Eigen::Vector2d a = 6 * (1 - t) * (P0 - 2 * P1 + P2) + 6 * t * (P1 - 2 * P2 + P3);

    path.t[i] = t;
    path.x[i] = p.x();
    path.y[i] = p.y();
    path.theta[i] = atan2(v.y(), v.x());
    path.omega[i] = (v.x() * a.y() - v.y() * a.x()) / (v.x() * v.x() + v.y() * v.y());
  }

  path.calculateRemainingPathLengths();

  return path;
}

Trajectory generateQuadraticBezierCurve(
  Eigen::Vector2d start_point,
  Eigen::Vector2d end_point,
  double end_angle_rad,
  double lead,
  int num_points = 250)
{
  Trajectory path(num_points);

  const Eigen::Vector2d & P0 = start_point;
  const Eigen::Vector2d & P1 = end_point;
  const Eigen::Vector2d & PC = end_point - lead * Eigen::Vector2d(cos(end_angle_rad), sin(end_angle_rad));

  for (int i = 0; i < num_points; i++) {
    double t = static_cast<double>(i) / static_cast<double>(num_points);
    Eigen::Vector2d p = pow(1 - t, 2) * P0 + 2 * (1 - t) * t * PC + t * t * P1;
    Eigen::Vector2d v = 2 * (1 - t) * (PC - P0) + 2 * t * (P1 - PC);
    Eigen::Vector2d a = 2 * (P1 - 2 * PC + P0);

    path.t[i] = t;
    path.x[i] = p.x();
    path.y[i] = p.y();
    path.theta[i] = atan2(v.y(), v.x());
    path.omega[i] = (v.x() * a.y() - v.y() * a.x()) / (v.x() * v.x() + v.y() * v.y());
  }

  path.calculateRemainingPathLengths();

  return path;
}

} // namespace motion_planning
} // namespace ghost_tank
