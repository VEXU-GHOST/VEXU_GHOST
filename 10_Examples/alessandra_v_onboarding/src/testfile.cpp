#include <gtest/gtest.h>
#include "try.h"

TEST(MyFunctions, AddTest) {
  MyClass obj;
  EXPECT_EQ(obj.add(), 30);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
