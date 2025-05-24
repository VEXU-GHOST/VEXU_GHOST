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

#include <ghost_tank/motion_planning/trajectory.hpp>
#include <gtest/gtest.h>

using ghost_tank::motion_planning::Trajectory;

TEST(TestTrajectory, testConstructors) {
  EXPECT_NO_THROW(auto traj = Trajectory());
}

TEST(TestTrajectory, testResize) {
  auto traj = Trajectory();

  EXPECT_EQ(traj.size(), 0);

  traj.resize(10);
  EXPECT_EQ(traj.size(), 10);

  EXPECT_EQ(traj.size(), 10);

  EXPECT_EQ(traj.t.size(), 10);
  EXPECT_EQ(traj.x.size(), 10);
  EXPECT_EQ(traj.y.size(), 10);
  EXPECT_EQ(traj.theta.size(), 10);
  EXPECT_EQ(traj.omega.size(), 10);
  EXPECT_EQ(traj.remaining_path_length.size(), 10);
}

TEST(TestTrajectory, testClear) {
  auto traj = Trajectory();

  traj.resize(10);
  traj.clear();

  EXPECT_EQ(traj.t.size(), 0);
  EXPECT_EQ(traj.x.size(), 0);
  EXPECT_EQ(traj.y.size(), 0);
  EXPECT_EQ(traj.theta.size(), 0);
  EXPECT_EQ(traj.omega.size(), 0);
  EXPECT_EQ(traj.remaining_path_length.size(), 0);
}

TEST(TestTrajectory, testCalculateRemainingPathLengths) {
  auto traj = Trajectory();

  int num_segments = 10;
  traj.resize(num_segments);
  for (double i = 0; i < static_cast<double>(num_segments); i += 1.0) {
    traj.t[i] = i / num_segments;
    traj.x[i] = 3.0 * i;
    traj.y[i] = 4.0 * i;
  }

  traj.calculateRemainingPathLengths();

  int count = 0;
  for (int i = traj.t.size() - 1; i >= 0; i--) {
    EXPECT_EQ(traj.remaining_path_length[i], count * 5.0);
    count++;
  }
}
