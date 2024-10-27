#include "ghost_tank/tank_odom.hpp"
#include "gtest/gtest.h"

class TestTankOdom : public ::testing::Test
{
public:
  TestTankOdom()
  {
  }
};

TEST_F(TestTankOdom, testInterpolate) {
  Eigen::Vector3d got, expected;
  ghost_tank::TankOdometry t(100, 1. / 2. / M_PI, 10);
  t.setPose({0,0,0});

    got = t.update({100}, {100}, 0);
    expected = {1, 0, 0};
    EXPECT_TRUE((got - expected).norm() < 0.0001)  << "expected: {" << expected.transpose() << "} got: {" << got.transpose() << "}" << std::endl;

    got = t.update({100}, {(long) (100 + M_PI/2 * 10 * 100)}, M_PI/2);
    expected = {6, 5, M_PI/2};
    EXPECT_TRUE((got - expected).norm() < 0.01)  << "expected: {" << expected.transpose() << "} got: {" << got.transpose() << "}" << std::endl;

    got = t.update({300}, {(long) (100 + M_PI/2 * 10 * 100) + 200}, M_PI/2);
    expected = {8, 5, M_PI/2};
    EXPECT_TRUE((got - expected).norm() < 0.01)  << "expected: {" << expected.transpose() << "} got: {" << got.transpose() << "}" << std::endl;
  }

