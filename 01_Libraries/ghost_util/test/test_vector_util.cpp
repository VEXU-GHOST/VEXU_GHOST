#include "ghost_util/vector_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;

class TestAngleUtil : public ::testing::Test {
protected:

	void SetUp() override {
	}
};

TEST_F(TestAngleUtil, testAngleBetweenVectorsDegrees){
	EXPECT_DOUBLE_EQ(180.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(0, 1)));
	EXPECT_DOUBLE_EQ(135.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
	EXPECT_DOUBLE_EQ(90.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(1, 0)));
	EXPECT_DOUBLE_EQ(45.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
}

TEST_F(TestAngleUtil, testAngleBetweenVectorsRadians){
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 180.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(0, 1)));
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 135.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 90.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(1, 0)));
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 45.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}