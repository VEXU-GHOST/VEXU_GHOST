#include <gtest/gtest.h>
#include "linh-onboarding/subscriber.hpp"
#include <iostream>


class TestExample : public ::testing::Test {
protected:

	TestExample(){
		subscriber_node_ = std::make_shared<onboarding::Subscriber>();
	}

	void SetUp() override {
	}

	void TearDown() override {
	}
	std::shared_ptr<onboarding::Subscriber> subscriber_node_;
};

TEST_F(TestExample, testAdd){
	EXPECT_EQ(subscriber_node_->add(2, 3), 5);
	EXPECT_TRUE(subscriber_node_->add(2, 3) == 5);
	EXPECT_FALSE(subscriber_node_->add(2, 3) == 4);
}

TEST_F(TestExample, testSubtract){
	EXPECT_EQ(subscriber_node_->subtract(2, 3), -1);
    EXPECT_TRUE(subscriber_node_->subtract(5, 3) == 2);
	EXPECT_FALSE(subscriber_node_->subtract(2, 3) == 4);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
