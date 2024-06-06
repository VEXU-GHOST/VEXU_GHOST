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

#include "ghost_util/math_util.hpp"

namespace ghost_util
{

bool isPositive(double val)
{
  return val >= 0.0;
}

double slewRate(double curr, double next, double limit)
{
  limit = fabs(limit);
  if (next > curr + limit) {
    return curr + limit;
  } else if (next < curr - limit) {
    return curr - limit;
  } else {
    return next;
  }
}

double sign(double val)
{
  return isPositive(val) ? 1.0 : -1.0;
}

std::vector<double> clampedVectorInterpolate(
  double x_new, double x1, double x2,
  const std::vector<double> & v1,
  const std::vector<double> & v2)
{
  // Check sizes
  if (v1.size() != v2.size()) {
    throw std::runtime_error(
            "[clampedVectorInterpolate] Error: Vectors must be the same size!");
  }

  if (v1.empty()) {
    throw std::runtime_error(
            "[clampedVectorInterpolate] Error: Vectors cannot be empty!");
  }

  std::vector<double> v_new;
  if (x_new <= x1) {
    v_new = v1;
  } else if (x_new >= x2) {
    v_new = v2;
  } else {
    double frac = (x_new - x1) / (x2 - x1);
    v_new.reserve(v1.size());

    for (int i = 0; i < v1.size(); i++) {
      v_new.push_back((1 - frac) * v1[i] + frac * v2[i]);
    }
  }
  return v_new;
}

double linearInterpolate(
  const std::vector<double> & x_data,
  const std::vector<double> & y_data,
  const double desired_x)
{
  // check if empty
  if ((x_data.size() == 0) || (y_data.size() == 0)) {
    throw std::runtime_error("[linearInterpolate] Error: cannot interpolate empty vector");
  }

  // check vector lengths
  if (x_data.size() != y_data.size()) {
    throw std::runtime_error("[linearInterpolate] Error: size of input vectors are unequal");
  }

  // get upper and lower bounds as an index
  auto upper_index = std::upper_bound(x_data.begin(), x_data.end(), desired_x) - x_data.begin();
  auto lower_index = std::lower_bound(x_data.begin(), x_data.end(), desired_x) - x_data.begin();

  // return exact match
  if (upper_index != lower_index) {
    return y_data[lower_index];
  }

  // extrapolate from first two points if desired x before range
  if (upper_index == 0) {
    return ((y_data[1] - y_data[0]) / (x_data[1] - x_data[0])) *
           (desired_x - x_data[0]) + y_data[0];
  }

  // extrapolate from last two points if desired x after range
  if (lower_index == x_data.size()) {
    auto y_last = y_data.end() - 1;
    auto x_last = x_data.end() - 1;
    return ((*y_last - *(y_last - 1)) / (*x_last - *(x_last - 1))) *
           (desired_x - *x_last) + *y_last;
  }

  // interpolate
  return ((y_data[upper_index] - y_data[lower_index - 1]) /
         (x_data[upper_index] - x_data[lower_index - 1])) *
         (desired_x - x_data[lower_index - 1]) + y_data[lower_index - 1];
}

double clampedLinearInterpolate(
  const std::vector<double> & x_data,
  const std::vector<double> & y_data,
  const double desired_x)
{
  if (desired_x <= x_data.front()) {
    return y_data.front();
  }

  if (desired_x >= x_data.back()) {
    return y_data.back();
  }

  return linearInterpolate(x_data, y_data, desired_x);
}

} // namespace ghost_util
