#include "ghost_util/read_path.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;


class TESTREAD : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }
};

TEST_F(TESTREAD, testReadstuff) {

  std::vector<double> expected_x = {9.0, 11.0, 14.0, 1.0};
  std::vector<double> expected_y = {10.0, 12.0, 15.0, 2.0};
  std::vector<double> expected_theta = {11.0, 13.0, 16.0, 3.0};
  std::vector<double> x_values;
  std::vector<double> y_values;
  std::vector<double> angle_values;
  

  

    readPathFromFile("/home/johnny/VEXU_GHOST/01_Libraries/ghost_util/test/config/path.txt",x_values, y_values, angle_values);

    EXPECT_EQ(x_values, expected_x);
    EXPECT_EQ(y_values, expected_y);
    EXPECT_EQ(angle_values, expected_theta);
  
}
TEST_F(TESTREAD, testfail) {

  std::vector<double> expected_x = {9.0, 11.0, 14.0, 1.0};
  std::vector<double> expected_y = {10.0, 12.0, 15.0, 2.0};
  std::vector<double> expected_theta = {11.0, 13.0, 16.0, 3.0};
  std::vector<double> x_values;
  std::vector<double> y_values;
  std::vector<double> angle_values;
  

  

    EXPECT_EQ(readPathFromFile("fdxhtrtrd",x_values, y_values, angle_values),1);

  
}
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
