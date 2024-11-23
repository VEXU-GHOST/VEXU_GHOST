#include "ghost_tank/tank_odom.hpp"
#include "gtest/gtest.h"
#include <ghost_util/unit_conversion_utils.hpp>

class TestTankOdom : public ::testing::Test
{
public:
  TestTankOdom()
  {
  }
};

#define GOT_EXPECTED_EXPECT(n) \
  EXPECT_TRUE( \
    (got - expected).norm() < \
    n) << "expected: {" << expected.transpose() << "} got: {" << got.transpose() << "}" << \
    std::endl;
TEST_F(TestTankOdom, testSimple) {
  Eigen::Vector3d got, expected;
  ghost_tank::TankOdometry t(100, 1. / 2. / M_PI, 10);
  t.setPose({0, 0, 0});

  got = t.update({100}, {100});
  expected = {1, 0, 0};
  GOT_EXPECTED_EXPECT(0.0001)

  got = t.update({100}, {(long) (100 + M_PI / 2 * 10 * 100)});
  expected = {6, 5, M_PI / 2};
  GOT_EXPECTED_EXPECT(0.01)

  got = t.update({300}, {(long) (100 + M_PI / 2 * 10 * 100) + 200});
  expected = {6, 7, M_PI / 2};
  GOT_EXPECTED_EXPECT(0.01)
}

TEST_F(TestTankOdom, testTriangle) {
  Eigen::Vector3d got, expected;
  ghost_tank::TankOdometry t(100, 1. / 2. / M_PI, 10. * 2 / M_PI);

  std::vector<Eigen::Vector2i> encoder_diffs = {
    {0, 0},
    {100, 100},
    {100 * 10 * 3 / 4, -100 * 10 * 3 / 4},
    {100 * sqrt(2), 100 * sqrt(2)},
    {100 * 10 * -5 / 4, -100 * 10 * -5 / 4},
    {100,100},
    {100 * 10 * 2 / 4, -100 * 10 * 2 / 4},
  };
  std::vector<Eigen::Vector3d> expected_pts = {
    {0, 0, 0},
    {1, 0, 0},
    {1, 0, (360 - 135) * ghost_util::DEG_TO_RAD},
    {0, -1, (360 - 135) * ghost_util::DEG_TO_RAD},
    {0, -1, 90 * ghost_util::DEG_TO_RAD},
    {0, 0, 90  * ghost_util::DEG_TO_RAD   },
    {0, 0, 0},
  };


  for (int i = 0; i < expected_pts.size(); i++) {
    ASSERT_LT(i, encoder_diffs.size());
    if (i >= 1) {
      encoder_diffs[i] += encoder_diffs[i - 1];            //accumlate the encoder values
    }
    got = t.update({encoder_diffs[i].x()}, {encoder_diffs[i].y()});
    expected = expected_pts[i];
    GOT_EXPECTED_EXPECT(0.01);
  }

}
