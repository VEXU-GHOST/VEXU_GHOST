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
    if (t.size() != x.size() || t.size() != y.size()) {
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

  std::vector<double> t;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  std::vector<double> omega;
  std::vector<double> remaining_path_length;
};

} //namespace motion_planning
} //namespace ghost_tank
