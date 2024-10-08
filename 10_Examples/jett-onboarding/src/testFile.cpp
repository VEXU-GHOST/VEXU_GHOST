#include "gtest/gtest.h"
#include "talking.h"
#include "string.h"

class TestExample : public ::testing::Test
{
protected:
  TestExample()
  {
    talking_ = std::make_shared<talking>();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

  std::shared_ptr<talking> talking_;

};

TEST_F(TestExample, testTalk) {
  //testing the talk function
  EXPECT_EQ("Test", talking_->talk("Test"));
}

TEST_F(TestExample, testNumberPrint) {
  EXPECT_TRUE(talking_->numberPrint() == talking_->num1);
}

TEST_F(TestExample, testTwoToPower) {
  EXPECT_EQ(talking_->twoToPower(5) == (2 ^ 5));
}


// int main(int argc, char ** argv)
// {
//   ::testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
