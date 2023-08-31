#include "ghost_common/util/angle_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_common;

class TestAngleUtil : public ::testing::Test {
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

TEST_F(TestAngleUtil, testWrapAngle2PI){
	EXPECT_NEAR(0.2,    WrapAngle2PI(0.2), 0.01);
	EXPECT_NEAR(0.2,    WrapAngle2PI(2 * M_PI + 0.2), 0.01);
	EXPECT_NEAR(0.2,    WrapAngle2PI(4 * M_PI + 0.2), 0.01);
	EXPECT_NEAR(0.2,    WrapAngle2PI(-2 * M_PI + 0.2), 0.01);
	EXPECT_NEAR(0.2,    WrapAngle2PI(-4 * M_PI + 0.2), 0.01);
	EXPECT_NEAR(M_PI + 0.01,  WrapAngle2PI(M_PI + 0.01), 0.01);
	EXPECT_NEAR(M_PI - 0.01,  WrapAngle2PI(-M_PI - 0.01), 0.01);
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

TEST_F(TestAngleUtil, testWrapAnglePI){
	EXPECT_NEAR(0.2,            WrapAnglePI(0.2), 0.01);

	EXPECT_NEAR(M_PI - 0.2,     WrapAnglePI(M_PI - 0.2), 0.01);
	EXPECT_NEAR(-(M_PI - 0.2),  WrapAnglePI(-(M_PI - 0.2)), 0.01);

	EXPECT_NEAR(-(M_PI - 0.2),  WrapAnglePI(M_PI + 0.2), 0.01);
	EXPECT_NEAR(M_PI - 0.2,     WrapAnglePI(-(M_PI + 0.2)), 0.01);

	EXPECT_NEAR(-M_PI + 0.2,    WrapAnglePI(M_PI + 0.2), 0.01);
	EXPECT_NEAR(-M_PI + 0.2,    WrapAnglePI(M_PI + 0.2 + 2 * M_PI), 0.01);

	EXPECT_NEAR(M_PI - 0.2,     WrapAnglePI(-M_PI - 0.2), 0.01);
	EXPECT_NEAR(M_PI - 0.2,     WrapAnglePI(-M_PI - 0.2 - 2 * M_PI), 0.01);
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

TEST_F(TestAngleUtil, testFlipAnglePI){
	EXPECT_NEAR(M_PI - 0.2,  FlipAnglePI(-0.2), 0.01);
	EXPECT_NEAR(-0.2,   FlipAnglePI(M_PI - 0.2), 0.01);

	EXPECT_NEAR(-M_PI + 0.2, FlipAnglePI(0.2), 0.01);
	EXPECT_NEAR(0.2,    FlipAnglePI(-M_PI + 0.2), 0.01);

	EXPECT_NEAR(-3 * M_PI / 4, FlipAnglePI(M_PI / 4), 0.01);
	EXPECT_NEAR(M_PI / 4,   FlipAnglePI(-3 * M_PI / 4), 0.01);

	EXPECT_NEAR(3 * M_PI / 4,  FlipAnglePI(-M_PI / 4), 0.01);
	EXPECT_NEAR(-M_PI / 4,  FlipAnglePI(3 * M_PI / 4), 0.01);
}

TEST_F(TestAngleUtil, testSmallestAngleDistDeg){
	EXPECT_NEAR(2.0,    SmallestAngleDistDeg(3.0, 1.0), 0.01);
	EXPECT_NEAR(2.0,    SmallestAngleDistDeg(1.0, -1.0), 0.01);

	EXPECT_NEAR(-90.0,  SmallestAngleDistDeg(1, 91.0), 0.01);
	EXPECT_NEAR(90.0,   SmallestAngleDistDeg(91.0, 1.0), 0.01);

	EXPECT_NEAR(-178.0, SmallestAngleDistDeg(91.0, -91.0), 0.01);
	EXPECT_NEAR(178.0,  SmallestAngleDistDeg(-91.0, 91.0), 0.01);

	EXPECT_NEAR(-2.0,   SmallestAngleDistDeg(179.0, -179.0), 0.01);
	EXPECT_NEAR(2.0,    SmallestAngleDistDeg(-179.0, 179.0), 0.01);

	EXPECT_NEAR(-2.0,   SmallestAngleDistDeg(359.0, 1.0), 0.01);
	EXPECT_NEAR(2.0,    SmallestAngleDistDeg(1.0, 359.0), 0.01);
}

TEST_F(TestAngleUtil, testSmallestAngleDistRad){
	EXPECT_NEAR(0.5,    SmallestAngleDistRad(0.7, 0.2), 0.01);
	EXPECT_NEAR(0.4,    SmallestAngleDistRad(0.2, -0.2), 0.01);

	EXPECT_NEAR(-M_PI / 2,  SmallestAngleDistRad(0.2, M_PI / 2 + 0.2), 0.01);
	EXPECT_NEAR(M_PI / 2,   SmallestAngleDistRad(M_PI / 2 + 0.2, 0.2), 0.01);

	EXPECT_NEAR(-M_PI + 0.4, SmallestAngleDistRad((M_PI / 2 + 0.2), -(M_PI / 2 + 0.2)), 0.01);
	EXPECT_NEAR(M_PI - 0.4,  SmallestAngleDistRad(-(M_PI / 2 + 0.2), M_PI / 2 + 0.2), 0.01);

	EXPECT_NEAR(-0.4,   SmallestAngleDistRad(M_PI - 0.2, -M_PI + 0.2), 0.01);
	EXPECT_NEAR(0.4,    SmallestAngleDistRad(-M_PI + 0.2, M_PI - 0.2), 0.01);

	EXPECT_NEAR(-0.4,   SmallestAngleDistRad(2 * M_PI - 0.2, 0.2), 0.01);
	EXPECT_NEAR(0.4,    SmallestAngleDistRad(0.2, 2 * M_PI - 0.2), 0.01);
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}