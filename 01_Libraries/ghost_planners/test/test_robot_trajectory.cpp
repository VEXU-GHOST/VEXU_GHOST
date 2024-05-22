#include <ghost_planners/robot_trajectory.hpp>
#include <gtest/gtest.h>

using ghost_planners::RobotTrajectory;

class RobotTrajectoryTestFixture : public ::testing::Test
{
public:
  RobotTrajectoryTestFixture()
  {
  }

  void SetUp() override
  {
  }
};

TEST_F(RobotTrajectoryTestFixture, testConstructors) {
  EXPECT_NO_THROW(auto traj = RobotTrajectory());
}
