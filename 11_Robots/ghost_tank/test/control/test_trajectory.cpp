/*
 * Copyright (c) 2025 Maxx Wilson
 * All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <gtest/gtest.h>
#include "ghost_tank/control/trajectory.hpp" // Path to your Trajectory.hpp
#include <eigen3/Eigen/Core> // For Eigen::Vector2d
#include <vector>
#include <cmath> // For std::sqrt, std::abs
#include <limits> // For std::numeric_limits

using namespace ghost_tank::motion_planning;

// Helper function for floating point comparisons
const double EPSILON = 1e-9;

// Test fixture for Trajectory operations
class TrajectoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Common setup if needed for multiple tests
  }

  void TearDown() override
  {
    // Common teardown if needed
  }
};

TEST_F(TrajectoryTest, DefaultConstructorInitializesEmpty) {
  Trajectory traj;
  EXPECT_EQ(traj.size(), 0);
  EXPECT_TRUE(traj.x.empty());
  EXPECT_TRUE(traj.y.empty());
  EXPECT_TRUE(traj.theta.empty());
  EXPECT_TRUE(traj.t.empty());
  EXPECT_TRUE(traj.omega.empty());
  EXPECT_TRUE(traj.remaining_path_length.empty());
}

TEST_F(TrajectoryTest, ConstructorWithSizeInitializesCorrectly) {
  int expected_size = 5;
  Trajectory traj(expected_size);
  EXPECT_EQ(traj.size(), expected_size);
  EXPECT_EQ(traj.x.size(), expected_size);
  EXPECT_EQ(traj.y.size(), expected_size);
  EXPECT_EQ(traj.theta.size(), expected_size);
  EXPECT_EQ(traj.t.size(), expected_size);
  EXPECT_EQ(traj.omega.size(), expected_size);
  EXPECT_EQ(traj.remaining_path_length.size(), expected_size);

  // Check if elements are initialized to 0.0
  for (int i = 0; i < expected_size; ++i) {
    EXPECT_NEAR(traj.x[i], 0.0, EPSILON);
    EXPECT_NEAR(traj.y[i], 0.0, EPSILON);
    EXPECT_NEAR(traj.theta[i], 0.0, EPSILON);
    EXPECT_NEAR(traj.t[i], 0.0, EPSILON);
    EXPECT_NEAR(traj.omega[i], 0.0, EPSILON);
    EXPECT_NEAR(traj.remaining_path_length[i], 0.0, EPSILON);
  }
}

TEST_F(TrajectoryTest, ResizeMethodWorks) {
  Trajectory traj;
  traj.resize(3);
  EXPECT_EQ(traj.size(), 3);
  traj.resize(7);
  EXPECT_EQ(traj.size(), 7);
  traj.resize(0);
  EXPECT_EQ(traj.size(), 0);
}

TEST_F(TrajectoryTest, ClearMethodWorks) {
  Trajectory traj(5);
  traj.clear();
  EXPECT_EQ(traj.size(), 0);
  EXPECT_TRUE(traj.x.empty());
}

TEST_F(TrajectoryTest, CalculateRemainingPathLengthsStraightLine) {
  Trajectory traj(3);
  traj.x = {0.0, 1.0, 2.0};
  traj.y = {0.0, 0.0, 0.0};
  traj.theta = {0.0, 0.0, 0.0};   // Not directly used in length calculation, but part of traj

  EXPECT_TRUE(traj.calculateRemainingPathLengths());

  // Path segment lengths:
  // (0,0) to (1,0) = 1.0
  // (1,0) to (2,0) = 1.0
  // Total path length = 2.0

  EXPECT_NEAR(traj.remaining_path_length[0], 2.0, EPSILON);   // Length from start to end
  EXPECT_NEAR(traj.remaining_path_length[1], 1.0, EPSILON);   // Length from 2nd point to end
  EXPECT_NEAR(traj.remaining_path_length[2], 0.0, EPSILON);   // Length from last point to end
}

TEST_F(TrajectoryTest, CalculateRemainingPathLengthsAngledLine) {
  Trajectory traj(3);
  traj.x = {0.0, 1.0, 1.0};
  traj.y = {0.0, 0.0, 1.0};
  traj.theta = {0.0, 0.0, 0.0};

  EXPECT_TRUE(traj.calculateRemainingPathLengths());

  // Path segment lengths:
  // (0,0) to (1,0) = 1.0
  // (1,0) to (1,1) = 1.0
  // Total path length = 2.0

  EXPECT_NEAR(traj.remaining_path_length[0], 2.0, EPSILON);
  EXPECT_NEAR(traj.remaining_path_length[1], 1.0, EPSILON);
  EXPECT_NEAR(traj.remaining_path_length[2], 0.0, EPSILON);
}

TEST_F(TrajectoryTest, CalculateRemainingPathLengthsDiagonalLine) {
  Trajectory traj(3);
  traj.x = {0.0, 1.0, 2.0};
  traj.y = {0.0, 1.0, 2.0};
  traj.theta = {0.0, 0.0, 0.0};

  EXPECT_TRUE(traj.calculateRemainingPathLengths());

  // Path segment lengths:
  // (0,0) to (1,1) = sqrt(2) approx 1.414
  // (1,1) to (2,2) = sqrt(2) approx 1.414
  // Total path length = 2 * sqrt(2) approx 2.828

  EXPECT_NEAR(traj.remaining_path_length[0], 2 * std::sqrt(2.0), EPSILON);
  EXPECT_NEAR(traj.remaining_path_length[1], std::sqrt(2.0), EPSILON);
  EXPECT_NEAR(traj.remaining_path_length[2], 0.0, EPSILON);
}


TEST_F(TrajectoryTest, CalculateRemainingPathLengthsEmptyTrajectory) {
  Trajectory traj;
  EXPECT_FALSE(traj.calculateRemainingPathLengths());
  EXPECT_TRUE(traj.remaining_path_length.empty());
}

TEST_F(TrajectoryTest, CalculateRemainingPathLengthsSinglePoint) {
  Trajectory traj(1);
  traj.x = {0.0};
  traj.y = {0.0};
  EXPECT_TRUE(traj.calculateRemainingPathLengths());
  EXPECT_EQ(traj.remaining_path_length.size(), 1);
  EXPECT_NEAR(traj.remaining_path_length[0], 0.0, EPSILON);   // A single point has 0 remaining length
}

TEST_F(TrajectoryTest, CalculateRemainingPathLengthsMismatchedDimensions) {
  Trajectory traj;
  traj.x.resize(3);
  traj.y.resize(2);   // Mismatch
  traj.t.resize(3);
  // This should print an error to console and return false
  EXPECT_FALSE(traj.calculateRemainingPathLengths());
}

// -----------------------------------------------------------
// Tests for getIndexOfClosestPoint
// -----------------------------------------------------------

TEST_F(TrajectoryTest, getIndexOfClosestPointEmptyTrajectory) {
  Trajectory traj;
  Eigen::Vector2d pos(0.0, 0.0);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), -1);
}

TEST_F(TrajectoryTest, getIndexOfClosestPointSinglePointTrajectory) {
  Trajectory traj(1);
  traj.x = {1.0};
  traj.y = {2.0};
  Eigen::Vector2d pos(1.0, 2.0);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 0);
  pos = Eigen::Vector2d(1.1, 2.1);   // Slightly off
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 0);
}

TEST_F(TrajectoryTest, getIndexOfClosestPointExactMatch) {
  Trajectory traj(3);
  traj.x = {0.0, 1.0, 2.0};
  traj.y = {0.0, 1.0, 2.0};

  Eigen::Vector2d pos(1.0, 1.0);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 1);
}

TEST_F(TrajectoryTest, getIndexOfClosestPointBetweenPoints) {
  Trajectory traj(3);
  traj.x = {0.0, 1.0, 2.0};
  traj.y = {0.0, 0.0, 0.0};   // Straight line on x-axis

  Eigen::Vector2d pos(0.6, 0.1);   // Closer to (1.0, 0.0) than (0.0, 0.0)
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 1);

  pos = Eigen::Vector2d(0.4, 0.1);   // Closer to (0.0, 0.0) than (1.0, 0.0)
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 0);

  pos = Eigen::Vector2d(1.4, -0.1);   // Closer to (1.0, 0.0) than (2.0, 0.0)
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 1);

  pos = Eigen::Vector2d(1.6, -0.1);   // Closer to (2.0, 0.0) than (1.0, 0.0)
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 2);
}

TEST_F(TrajectoryTest, getIndexOfClosestPointOffTrajectory) {
  Trajectory traj(3);
  traj.x = {0.0, 1.0, 2.0};
  traj.y = {0.0, 0.0, 0.0};

  Eigen::Vector2d pos(5.0, 5.0);   // Far from all points
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 2);   // Should find the closest end point
}

TEST_F(TrajectoryTest, getIndexOfClosestPointCurvedTrajectory) {
  Trajectory traj(4);
  traj.x = {0.0, 1.0, 1.0, 0.0};
  traj.y = {0.0, 0.0, 1.0, 1.0};

  Eigen::Vector2d pos(0.5, 0.5);   // Center of the square path
  // Distances:
  // (0,0): sqrt(0.5^2 + 0.5^2) = sqrt(0.5) = 0.707
  // (1,0): sqrt(0.5^2 + 0.5^2) = 0.707
  // (1,1): sqrt(0.5^2 + 0.5^2) = 0.707
  // (0,1): sqrt(0.5^2 + 0.5^2) = 0.707
  // If all distances are equal, it should return the first one found (index 0 for (0,0))
  // No, it should return the one it hits first among equals.
  // However, floating point precision might make one slightly smaller.
  // Let's test points clearly closer to one specific point.

  pos = Eigen::Vector2d(0.1, 0.1);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 0);   // Closest to (0,0)

  pos = Eigen::Vector2d(0.9, 0.1);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 1);   // Closest to (1,0)

  pos = Eigen::Vector2d(0.9, 0.9);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 2);   // Closest to (1,1)

  pos = Eigen::Vector2d(0.1, 0.9);
  EXPECT_EQ(traj.getIndexOfClosestPoint(pos), 3);   // Closest to (0,1)
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
