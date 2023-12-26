#include "gtest/gtest.h"
#include "will_onboarding1/Mather.hpp"

class MyTest : public ::testing::Test {
protected:

	MyTest(){
		node_ = std::make_shared<Math_Space::Mather>(5,2);
	}

	void SetUp() override {
	}

	void TearDown() override {
	}
	std::shared_ptr< Math_Space::Mather> node_;
};

TEST_F(MyTest, addTest){
	EXPECT_EQ((node_->decode(1)),7);
}
TEST_F(MyTest, multTest){
	EXPECT_EQ((node_->decode(2)),10);
}
TEST_F(MyTest, subTest){
	EXPECT_EQ((node_->decode(3)),3);
}
TEST_F(MyTest, modTest){
	EXPECT_EQ((node_->decode(4)),1);
}
TEST_F(MyTest, randTest){
	EXPECT_EQ((node_->decode(5)),0);
}

int main(int argc, char **argv) {
	// This line is always required if you are going to instantiate a ROS node.
	// For testing regular C++ libraries, it should be excluded.
	rclcpp::init(argc, argv);
	// Standard GTest main
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}