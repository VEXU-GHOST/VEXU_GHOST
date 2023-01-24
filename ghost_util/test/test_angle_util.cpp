#include "gtest/gtest.h"
#include "ghost_util/angle_util.hpp"

using namespace ghost_util;

class TestAngleUtil: public ::testing::Test {
    protected:

    void SetUp() override {
    }
};

TEST_F(TestAngleUtil, testWrapAngle360){
  EXPECT_NEAR(5.0,    WrapAngle360(5.0), 0.01);
  EXPECT_NEAR(5.0,    WrapAngle360(360.0 + 5.0), 0.01);
  EXPECT_NEAR(5.0,    WrapAngle360(360.0 + 360.0 + 5.0), 0.01);
  EXPECT_NEAR(5.0,    WrapAngle360(-360.0 + 5.0), 0.01);
  EXPECT_NEAR(5.0,    WrapAngle360(-360.0 - 360.0 + 5.0), 0.01);
  EXPECT_NEAR(181.0,  WrapAngle360(181.0), 0.01);
  EXPECT_NEAR(179.0,  WrapAngle360(-181.0), 0.01);
}

TEST_F(TestAngleUtil, testWrapAngle180){
  EXPECT_NEAR(1.0,    WrapAngle180(1.0), 0.01);

  EXPECT_NEAR(179.0,  WrapAngle180(179.0), 0.01);
  EXPECT_NEAR(-179.0, WrapAngle180(-179.0), 0.01);
  
  EXPECT_NEAR(-179.0, WrapAngle180(181.0), 0.01);
  EXPECT_NEAR(179.0,  WrapAngle180(-181.0), 0.01);

  EXPECT_NEAR(-175.0, WrapAngle180(185.0), 0.01);
  EXPECT_NEAR(-175.0, WrapAngle180(185.0 + 360.0), 0.01);

  EXPECT_NEAR(175.0,  WrapAngle180(-185.0), 0.01);
  EXPECT_NEAR(175.0,  WrapAngle180(-185.0 - 360.0), 0.01);
}


TEST_F(TestAngleUtil, testFlipAngle180){
  EXPECT_NEAR(179.0,  FlipAngle180(-1), 0.01);
  EXPECT_NEAR(-1.0,   FlipAngle180(179.0), 0.01);
  
  EXPECT_NEAR(-179.0, FlipAngle180(1), 0.01);
  EXPECT_NEAR(1.0,    FlipAngle180(-179.0), 0.01);
  
  EXPECT_NEAR(-135.0, FlipAngle180(45), 0.01);
  EXPECT_NEAR(45.0,   FlipAngle180(-135.0), 0.01);
 
  EXPECT_NEAR(135.0,  FlipAngle180(-45), 0.01);
  EXPECT_NEAR(-45.0,  FlipAngle180(135.0), 0.01);
}

TEST_F(TestAngleUtil, testSmallestAngleDist){
  EXPECT_NEAR(2.0,    SmallestAngleDist(3.0, 1.0), 0.01);
  EXPECT_NEAR(2.0,    SmallestAngleDist(1.0, -1.0), 0.01);

  EXPECT_NEAR(-90.0,  SmallestAngleDist(1, 91.0), 0.01);
  EXPECT_NEAR(90.0,   SmallestAngleDist(91.0, 1.0), 0.01);

  EXPECT_NEAR(-178.0, SmallestAngleDist(91.0, -91.0), 0.01);
  EXPECT_NEAR(178.0,  SmallestAngleDist(-91.0, 91.0), 0.01);

  EXPECT_NEAR(-2.0,   SmallestAngleDist(179.0, -179.0), 0.01);
  EXPECT_NEAR(2.0,    SmallestAngleDist(-179.0, 179.0), 0.01);
  
  EXPECT_NEAR(-2.0,   SmallestAngleDist(359.0, 1.0), 0.01);
  EXPECT_NEAR(2.0,    SmallestAngleDist(1.0, 359.0), 0.01);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}