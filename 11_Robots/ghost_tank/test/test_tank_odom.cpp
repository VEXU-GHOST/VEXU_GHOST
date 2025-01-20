#include "ghost_tank/tank_odom.hpp"
#include "gtest/gtest.h"
#include <ghost_util/unit_conversion_utils.hpp>

TEST(TestTankOdom, testSimple) {
  Eigen::Vector3d got, expected;
  ghost_tank::TankOdometry t(1000, 1. / 2. / M_PI, 10);
  t.setPose({0, 5, 0});

  got = t.update(1000, 1000);
  expected = {1, 5, 0};
  ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
    expected.transpose();


  got = t.update(-1000, -1000);
  expected = {0, 5, 0};
  ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
    expected.transpose();

  const int NUM_TURN_STEPS = 10;
  for (int i = 0; i < NUM_TURN_STEPS; i++) {
    got = t.update(-1000 * M_PI / 2, 0);
    //   steps over wheelbase * PI/2 for turning radius * 1000 for ticks
  }
  expected = {-5, 0, M_PI / 2};
  ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
    expected.transpose();

  got = t.update(1000, 1000);
  expected = {-5, 1, M_PI / 2};
  ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
    expected.transpose();
  got = t.update(-1000, -1000);
  expected = {-5, 0, M_PI / 2};
  ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
    expected.transpose();
}

TEST(TestTankOdom, testTriangle) {
  Eigen::Vector3d got, expected;
  ghost_tank::TankOdometry t(100, 1. / 2. / M_PI, 10. * 2 / M_PI);

  std::vector<Eigen::Vector2i> encoder_diffs = {
    {0, 0},
    {100, 100},
    {100 * 10 * 3 / 4, -100 * 10 * 3 / 4},
    {100 * sqrt(2), 100 * sqrt(2)},
    {100 * 10 * -5 / 4, -100 * 10 * -5 / 4},
    {100, 100},
    {100 * 10 * 2 / 4, -100 * 10 * 2 / 4},
  };
  std::vector<Eigen::Vector3d> expected_pts = {
    {0, 0, 0},
    {1, 0, 0},
    {1, 0, (360 - 135) * ghost_util::DEG_TO_RAD},
    {0, -1, (360 - 135) * ghost_util::DEG_TO_RAD},
    {0, -1, 90 * ghost_util::DEG_TO_RAD},
    {0, 0, 90 * ghost_util::DEG_TO_RAD},
    {0, 0, 0},
  };

  Eigen::Vector<long, 1> v_l(0);
  Eigen::Vector<long, 1> v_r(0);
  for (int i = 0; i < expected_pts.size(); i++) {
    ASSERT_LT(i, encoder_diffs.size());

    v_l[0] += encoder_diffs[i][0];
    v_r[0] += encoder_diffs[i][1];
    got = t.update(v_l, v_r);
    expected = expected_pts[i];
    ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
      expected.transpose();
  }
}


TEST(TestTankOdom, testTriangleInReverse) {
  Eigen::Vector3d got, expected;
  ghost_tank::TankOdometry t(100, 1. / 2. / M_PI, 10. * 2 / M_PI);

  std::vector<Eigen::Vector2i> encoder_diffs = {
    {-100 * 10 * 2 / 4, 100 * 10 * 2 / 4},
    {-100, -100},
    {-100 * 10 * -5 / 4, 100 * 10 * -5 / 4},
    {-100 * sqrt(2), -100 * sqrt(2)},
    {-100 * 10 * 3 / 4, 100 * 10 * 3 / 4},
    {-100, -100},
    {0, 0},
  };
  std::vector<Eigen::Vector3d> expected_pts = {
    {0, 0, 90 * ghost_util::DEG_TO_RAD},
    {0, -1, 90 * ghost_util::DEG_TO_RAD},
    {0, -1, (360 - 135) * ghost_util::DEG_TO_RAD},
    {1, 0, (360 - 135) * ghost_util::DEG_TO_RAD},
    {1, 0, 0},
    {0, 0, 0},
  };


  Eigen::Vector<long, 1> v_l(0);
  Eigen::Vector<long, 1> v_r(0);
  for (int i = 0; i < expected_pts.size(); i++) {
    ASSERT_LT(i, encoder_diffs.size());

    v_l[0] += encoder_diffs[i][0];
    v_r[0] += encoder_diffs[i][1];
    got = t.update(v_l, v_r);
    expected = expected_pts[i];
    ASSERT_TRUE((got - expected).isZero(0.01)) << "got: " << got.transpose() << " expected: " <<
      expected.transpose();
  }
}
