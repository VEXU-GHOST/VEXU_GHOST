#include <ghost_v5_core/device_config.hpp>
#include "gtest/gtest.h"

class TestExample : public ::testing::Test {
protected:
	void SetUp() override {
	}

	void TearDown() override {
	}
};

TEST_F(TestExample, test1){
	EXPECT_TRUE(true);
}