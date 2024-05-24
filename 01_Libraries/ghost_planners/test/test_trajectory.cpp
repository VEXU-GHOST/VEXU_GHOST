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

#include <ghost_planners/trajectory.hpp>
#include <gtest/gtest.h>

using ghost_planners::Trajectory;

TEST(TestTrajectory, testConstructors) {
  EXPECT_NO_THROW(auto traj = Trajectory(std::vector<std::string>{}));
}

TEST(TestTrajectory, testGetStateVectorSize) {
  auto name_vector = std::vector<std::string>{"s1", "s2", "s3"};
  auto trajectory = Trajectory(name_vector);
  EXPECT_EQ(trajectory.getStateNames(), name_vector);
}

TEST(TestTrajectory, testGetStateSize) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3"});
  EXPECT_EQ(trajectory.getStateVectorSize(), 3);
}

TEST(TestTrajectory, testGetStateIndex){
  auto state_names = std::vector<std::string>{"s1", "s2", "s3"};
  auto trajectory = Trajectory(state_names);

  int i = 0;
  for(const auto & name : state_names){
    EXPECT_EQ(trajectory.getStateIndex(name), i);
  }

}

TEST(TestTrajectory, testAddAndRetrieveNodes) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3", "s4"});
  EXPECT_NO_THROW(trajectory.addNode(0.0, std::vector<double>{0.0, 1.0, -2.0, -5.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{2.0, 0.0, 2.0, -10.0}));

  auto v1 = std::vector<double>{1.0, 0.5, 0.0, -7.5};
  EXPECT_EQ(trajectory.getNode(0.5), v1);
  auto v2 = std::vector<double>{0.0, 1.0, -2.0, -5.0};
  EXPECT_EQ(trajectory.getNode(-10.0), v2);
  auto v3 = std::vector<double>{2.0, 0.0, 2.0, -10.0};
  EXPECT_EQ(trajectory.getNode(10.0), v3);

  EXPECT_EQ(trajectory.getState("s2", 0.5), 0.5);
  EXPECT_EQ(trajectory.getState("s2", -10.0), 1.0);
  EXPECT_EQ(trajectory.getState("s2", 10.0), 0.0);

  auto v4 = std::vector<double>{1.0, 0.5, 0.0};
  EXPECT_EQ(
    trajectory.getStateTrajectory(
      "s2",
      std::vector<double>{-10.0, 0.5, 10.0}),
    v4
  );

}

TEST(TestTrajectory, testThrowsOnIncorrectNodeSize) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3"});
  EXPECT_THROW(trajectory.addNode(0.0, std::vector<double>{0.0, 1.0}), std::runtime_error);
}

TEST(TestTrajectory, testDuplicateTimeReplacesPreviousNode) {
    auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3"});
    trajectory.addNode(1.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});

    auto v1 = std::vector<double>{1.0, 2.0, -3.0, -7.0};
    trajectory.addNode(1.0, v1);

    EXPECT_EQ(trajectory.getNode(1.0), v1);
}

TEST(TestTrajectory, testInvalidStateNameThrows){
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3"});
  trajectory.addNode(0.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});
  EXPECT_THROW(trajectory.getStateIndex("not a state"), std::runtime_error);
}

