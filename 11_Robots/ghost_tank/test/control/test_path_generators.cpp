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

#include <ghost_tank/control/path_generators.hpp>
#include <ghost_util/unit_conversion_utils.hpp>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace ghost_tank::motion_planning;

void plot_path(Trajectory path)
{
  plt::figure();
  plt::plot(path.x, path.y, "r-");

  plt::figure();
  plt::subplot(3, 1, 1);
  plt::plot(path.t, path.theta);
  plt::subplot(3, 1, 2);
  plt::plot(path.t, path.omega);
  plt::subplot(3, 1, 3);
  plt::plot(path.t, path.remaining_path_length);
}

int main(int argc, char * argv[])
{
  {
    Eigen::Vector2d start(0.2, 0.4);
    Eigen::Vector2d end(1.0, 0.0);

    auto bezier_1 = generateCubicBezierCurve(
      start, 45.0 * ghost_util::DEG_TO_RAD,
      end, 90.0 * ghost_util::DEG_TO_RAD,
      1.0);
    plot_path(bezier_1);
  }

  {
    Eigen::Vector2d start(-0.5, -0.5);
    Eigen::Vector2d end(0.5, 0.5);

    auto bezier_2 = generateCubicBezierCurve(
      start, 0.0 * ghost_util::DEG_TO_RAD,
      end, -0.0 * ghost_util::DEG_TO_RAD,
      0.5);
    plot_path(bezier_2);
  }

  {
    Eigen::Vector2d start(-0.5, -0.5);
    Eigen::Vector2d end(0.5, 0.5);

    auto bezier_3 = generateQuadraticBezierCurve(start, end, -45.0 * ghost_util::DEG_TO_RAD, 1.0);
    plot_path(bezier_3);
  }

  plt::show();

}
