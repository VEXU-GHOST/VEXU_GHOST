#include "gtest/gtest.h"
#include "TrivialMethodsClass.h"

namespace {

TrivialMethodsClass trivialObj = TrivialMethodsClass();
TEST(TrivialTest, Test1) {
	EXPECT_EQ(16, trivialObj.test_method(4));
	EXPECT_EQ(1, trivialObj.test_method(-1));
}

}