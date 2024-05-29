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

TEST(TestTrajectory, testEmpty) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1"});
  EXPECT_TRUE(trajectory.empty());
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{2.0}));
  EXPECT_FALSE(trajectory.empty());
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

TEST(TestTrajectory, testGetStateIndex) {
  auto state_names = std::vector<std::string>{"s1", "s2", "s3"};
  auto trajectory = Trajectory(state_names);

  int i = 0;
  for (const auto & name : state_names) {
    EXPECT_EQ(trajectory.getStateIndex(name), i);
    i++;
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

  auto v5 = std::vector<double>{1.0, 0.75, 0.25, 0.0};
  EXPECT_EQ(
    trajectory.getStateTrajectory(
      "s2",
      std::vector<double>{-10.0, -0.25, 0.25, 10.0},
      0.5),
    v5
  );
}

TEST(TestTrajectory, testGetTimeVector) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1"});
  EXPECT_NO_THROW(trajectory.addNode(0.0, std::vector<double>{0.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{2.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.2, std::vector<double>{0.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.6, std::vector<double>{2.0}));

  auto expected = std::vector<double>{0.0, 1.0, 1.2, 1.6};
  EXPECT_EQ(trajectory.getTimeVector(), expected);
}

TEST(TestTrajectory, testClear) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1"});
  EXPECT_NO_THROW(trajectory.addNode(0.0, std::vector<double>{0.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{2.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.2, std::vector<double>{0.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.6, std::vector<double>{2.0}));

  trajectory.clearNodes();

  EXPECT_EQ(trajectory.getTimeVector().size(), 0);
}

TEST(TestTrajectory, testReset) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1"});
  EXPECT_NO_THROW(trajectory.addNode(0.0, std::vector<double>{0.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{2.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.2, std::vector<double>{0.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.6, std::vector<double>{2.0}));

  trajectory.reset(std::vector<std::string>{"s2", "s3"});
  auto expected = std::vector<std::string>{"s2", "s3"};
  EXPECT_EQ(trajectory.getStateNames(), expected);
  EXPECT_EQ(trajectory.getTimeVector().size(), 0);
}

TEST(TestTrajectory, testThrowsOnIncorrectNodeSize) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3"});
  EXPECT_THROW(trajectory.addNode(0.0, std::vector<double>{0.0, 1.0}), std::runtime_error);
}

TEST(TestTrajectory, testDuplicateTimeReplacesPreviousNode) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3", "s4"});
  trajectory.addNode(1.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});

  auto v1 = std::vector<double>{1.0, 2.0, -3.0, -7.0};
  trajectory.addNode(1.000000001, v1);

  EXPECT_EQ(trajectory.getNode(1.0), v1);
}

TEST(TestTrajectory, testInvalidStateNameThrows) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2", "s3", "s4"});
  trajectory.addNode(0.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});
  EXPECT_THROW(trajectory.getStateIndex("not a state"), std::runtime_error);
}

TEST(TestTrajectory, testEqualityOperator) {
  auto t1 = Trajectory(std::vector<std::string>{"s1", "s2", "s3", "s4"});
  t1.addNode(0.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});
  t1.addNode(1.0, std::vector<double>{0.0, 1.0, -2.0, -6.0});

  auto t2 = Trajectory(std::vector<std::string>{"s1", "s2", "s3", "s4"});
  t2.addNode(0.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});
  t2.addNode(1.0, std::vector<double>{0.0, 1.0, -2.0, -6.0});

  EXPECT_EQ(t1, t2);

  t2.addNode(1.0, std::vector<double>{0.0, 1.0, -2.0, -5.0});
  EXPECT_FALSE(t1 == t2);
}

TEST(TestTrajectory, testGetFlattenedTrajectory) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2"});
  EXPECT_NO_THROW(trajectory.addNode(0.0, std::vector<double>{5.0, 2.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{5.5, 2.5}));
  EXPECT_NO_THROW(trajectory.addNode(2.0, std::vector<double>{6.0, 3.0}));

  std::vector<double> flattened_trajectory;
  EXPECT_NO_THROW(
    flattened_trajectory =
    trajectory.getFlattenedTrajectory(std::vector<double>{0.5, 1.5}));

  std::vector<double> expected_trajectory{5.25, 2.25, 5.75, 2.75};

  EXPECT_EQ(flattened_trajectory, expected_trajectory);
}

TEST(TestTrajectory, testGetFlattenedTrajectoryTimeOffset) {
  auto trajectory = Trajectory(std::vector<std::string>{"s1", "s2"});
  EXPECT_NO_THROW(trajectory.addNode(0.0, std::vector<double>{5.0, 2.0}));
  EXPECT_NO_THROW(trajectory.addNode(1.0, std::vector<double>{5.5, 2.5}));
  EXPECT_NO_THROW(trajectory.addNode(2.0, std::vector<double>{6.0, 3.0}));

  std::vector<double> flattened_trajectory;
  EXPECT_NO_THROW(
    flattened_trajectory =
    trajectory.getFlattenedTrajectory(std::vector<double>{0.0, 1.0}, 0.5));

  std::vector<double> expected_trajectory{5.25, 2.25, 5.75, 2.75};

  EXPECT_EQ(flattened_trajectory, expected_trajectory);
}
