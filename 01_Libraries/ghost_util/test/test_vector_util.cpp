#include "ghost_util/vector_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;

class TestVectorUtil : public ::testing::Test {
protected:

	void SetUp() override {
	}
};

TEST_F(TestVectorUtil, testAngleBetweenVectorsDegrees){
	EXPECT_DOUBLE_EQ(180.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(0, 1)));
	EXPECT_DOUBLE_EQ(135.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
	EXPECT_DOUBLE_EQ(90.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(1, 0)));
	EXPECT_DOUBLE_EQ(45.0,  angleBetweenVectorsDegrees<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
}

TEST_F(TestVectorUtil, testAngleBetweenVectorsRadians){
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 180.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(0, 1)));
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 135.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, -1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 90.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(1, 0)));
	EXPECT_DOUBLE_EQ(DEG_TO_RAD * 45.0,  angleBetweenVectorsRadians<Eigen::Vector2d>(Eigen::Vector2d(0, 1),  Eigen::Vector2d(sqrt(2) / 2, sqrt(2) / 2)));
}

TEST_F(TestVectorUtil, testRotationMatrixIdentity){
	auto r = getRotationMatrixFromEulerAnglesDegrees(0.0, 0.0, 0.0);
	Eigen::Matrix3d expected;
	expected <<
	    1.0000000,  0.0000000,  0.0000000,
	    0.0000000,  1.0000000,  0.0000000,
	    0.0000000,  0.0000000,  1.0000000;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testRotationMatrixRoll45){
	auto r = getRotationMatrixFromEulerAnglesDegrees(45.0, 0.0, 0.0);
	Eigen::Matrix3d expected;
	expected <<
	    1.0000000,  0.0000000,  0.0000000,
	    0.0000000,  0.7071068, -0.7071068,
	    0.0000000,  0.7071068,  0.7071068;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testRotationMatrixRoll90){
	auto r = getRotationMatrixFromEulerAnglesDegrees(90.0, 0.0, 0.0);
	Eigen::Matrix3d expected;
	expected <<
	    1.0000000,  0.0000000,  0.0000000,
	    0.0000000,  0.0000000, -1.0000000,
	    0.0000000,  1.0000000,  0.0000000;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testRotationMatrixPitch45){
	auto r = getRotationMatrixFromEulerAnglesDegrees(0.0, 45.0, 0.0);
	Eigen::Matrix3d expected;
	expected <<
	    0.7071068,  0.0000000,  0.7071068,
	    0.0000000,  1.0000000,  0.0000000,
	    -0.7071068,  0.0000000,  0.7071068;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testRotationMatrixPitch90){
	auto r = getRotationMatrixFromEulerAnglesDegrees(0.0, 90.0, 0.0);
	Eigen::Matrix3d expected;
	expected <<
	    0.0000000,  0.0000000,  1.0000000,
	    0.0000000,  1.0000000,  0.0000000,
	    -1.0000000,  0.0000000,  0.0000000;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testRotationMatrixYaw45){
	auto r = getRotationMatrixFromEulerAnglesDegrees(0.0, 0.0, 45.0);
	Eigen::Matrix3d expected;
	expected <<
	    0.7071068, -0.7071068,  0.0000000,
	    0.7071068,  0.7071068,  0.0000000,
	    0.0000000,  0.0000000,  1.0000000;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testRotationMatrixYaw90){
	auto r = getRotationMatrixFromEulerAnglesDegrees(0.0, 0.0, 90.0);
	Eigen::Matrix3d expected;
	expected <<
	    0.0000000, -1.0000000,  0.0000000,
	    1.0000000,  0.0000000,  0.0000000,
	    0.0000000,  0.0000000,  1.0000000;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testCompositeRotationSimple){
	auto r = getRotationMatrixFromEulerAnglesDegrees(180.0, 45.0, 90.0);
	auto v = r * Eigen::Vector3d(1.0, 0.0, 0.0);
	Eigen::Vector3d expected(0.0, 0.7071068, -0.7071068);
	EXPECT_TRUE((v - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testCompositeRotation){
	auto r = getRotationMatrixFromEulerAnglesDegrees(22.0, 45.0, 60.0);
	Eigen::Matrix3d expected;
	expected <<
	    0.3535534, -0.6705213,  0.6522278,
	    0.6123725,  0.6929907,  0.3804785,
	    -0.7071068,  0.2648869,  0.6556180;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

TEST_F(TestVectorUtil, testCompositeRotationNegative){
	auto r = getRotationMatrixFromEulerAnglesDegrees(-25.87, -11.8, -40.0);
	Eigen::Matrix3d expected;
	expected <<
	    0.7498559,  0.6467240,  0.1395137,
	    -0.6292039,  0.6319218,  0.4525234,
	    0.2044961, -0.4271099,  0.8807715;
	EXPECT_TRUE((r - expected).norm() < 1e-6);
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}