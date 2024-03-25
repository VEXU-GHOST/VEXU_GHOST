#include "zaara-onboarding/trivialmethodssubscriber.hpp"
#include "gtest/gtest.h"

class OnboardingTest : public ::testing::Test {
protected:

	OnboardingTest(){
		subscriber_node_ = std::make_shared<myonboarding::TrivMethodSubscriber>();
	}

	void SetUp() override {
	}

	void TearDown() override {
	}
	std::shared_ptr<myonboarding::TrivMethodSubscriber> subscriber_node_;
};

//testing if addition of floats is working
TEST_F(OnboardingTest, testAddFloats){
	EXPECT_FLOAT_EQ(subscriber_node_->add_floats(2, 3), 5.0);
}

int main(int argc, char **argv) {
	
	rclcpp::init(argc, argv);

	
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}