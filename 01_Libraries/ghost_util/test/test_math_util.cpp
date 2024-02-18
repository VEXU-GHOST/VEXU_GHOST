#include "ghost_util/math_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;

class TestMathUtil : public ::testing::Test {
public:
	TestMathUtil(){
	}
};

TEST_F(TestMathUtil, testInterpolate){
	std::vector<double> x_data = {2.0, 3.0, 4.0};
	std::vector<double> y_data = {0.0};

	EXPECT_THROW(ghost_util::linearInterpolate(x_data, y_data, 2.0), std::runtime_error);

	y_data = {4.0, 6.0, 8.0};

	EXPECT_EQ(ghost_util::linearInterpolate(x_data, y_data, 1.0),  2.0); // < range
	EXPECT_EQ(ghost_util::linearInterpolate(x_data, y_data, 3.0),  6.0); // exact
	EXPECT_EQ(ghost_util::linearInterpolate(x_data, y_data, 5.0), 10.0); // > range
	EXPECT_EQ(ghost_util::linearInterpolate(x_data, y_data, 2.5),  5.0); // interpolate
}

TEST_F(TestMathUtil, testClampedInterpolate){
	std::vector<double> x_data = {2.0, 3.0, 4.0};
	std::vector<double> y_data = {4.0, 6.0, 8.0};

	EXPECT_EQ(ghost_util::clampedLinearInterpolate(x_data, y_data, 1.0), 4.0); // <= range
	EXPECT_EQ(ghost_util::clampedLinearInterpolate(x_data, y_data, 5.0), 8.0); // >= range
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}