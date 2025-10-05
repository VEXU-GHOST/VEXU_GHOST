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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "ghost_util/math_util.hpp"
#include <iostream>
#include <eigen3/Eigen/Core>

namespace ghost_tank
{

namespace motion_planning
{

struct Trajectory
{
  Trajectory(int size = 0)
  {
    resize(size);
  }

  void resize(int size)
  {
    t.resize(size, 0.0);
    x.resize(size, 0.0);
    y.resize(size, 0.0);
    theta.resize(size, 0.0);
    omega.resize(size, 0.0);
    remaining_path_length.resize(size, 0.0);
  }

  int size() const
  {
    return t.size();
  }

  void clear()
  {
    t.clear();
    x.clear();
    y.clear();
    theta.clear();
    omega.clear();
    remaining_path_length.clear();
  }

  bool calculateRemainingPathLengths()
  {
    if (t.empty() || t.size() != x.size() || t.size() != y.size()) {
      std::cout << "[ghost_tank::motion_planning::Trajectory::calculateRemainingPathLengths]" <<
        "Error: t, x, and y trajactories have mismatched dimension!" << std::endl;
      return false;
    }

    int num_points = t.size();
    remaining_path_length.resize(num_points);

    double path_len = 0.0;
    for (int i = remaining_path_length.size() - 1; i > 0; i--) {
      remaining_path_length[i] = path_len;

      const auto & curr_x = x[i];
      const auto & curr_y = y[i];
      const auto & prev_x = x[i - 1];
      const auto & prev_y = y[i - 1];

      path_len += sqrt(pow(curr_x - prev_x, 2) + pow(curr_y - prev_y, 2));
    }

    remaining_path_length[0] = path_len;
    return true;
  }

  /**
   * @brief Finds the index of the point on the trajectory closest to a given position.
   *
   * This method iterates through all points in the trajectory and calculates the
   * Euclidean distance to the `current_pos`. It returns the index of the
   * point with the minimum distance.
   *
   * @param current_pos The 2D Eigen::Vector2d representing the position to match.
   * @return The index of the closest point in the trajectory. Returns -1 if the
   * trajectory is empty.
   */
  int getIndexOfClosestPoint(const Eigen::Vector2d & current_pos) const
  {
    if (size() == 0) {
      return -1; // Return -1 if the trajectory is empty
    }

    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = 0;

    for (int i = 0; i < size(); ++i) {
      Eigen::Vector2d trajectory_point(x[i], y[i]);
      double dist = (trajectory_point - current_pos).norm();

      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }
    return closest_idx;
  }


  std::vector<double> t;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  std::vector<double> omega;
  std::vector<double> remaining_path_length;
  bool backwards{false};
};

} //namespace motion_planning
} //namespace ghost_tank
