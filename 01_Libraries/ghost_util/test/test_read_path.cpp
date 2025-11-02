#include "ghost_util/read_path.hpp"
#include "gtest/gtest.h"

class TESTREAD : public ::testing::Test {};

TEST_F(TESTREAD, testSuccess) {

    std::string test_filename_ = std::string(getenv("VEXU_HOME")) + "/""/01_Libraries/ghost_util/test/config/path.txt";

    std::vector<double> x_values = {0.0833, 0.0833, 0.0833, 3.0000, 5.9166, 5.9166, 5.9166};
    std::vector<double> y_values = {5.9166, 3.0000, 0.0833, 3.0000, 5.9166, 3.0000, 0.0833};
    std::vector<double> theta_values = {180, 180, 45, 45, 180, 180, 180};

    auto parsed = ghost_util::readPathFromFile(test_filename_);

    ASSERT_EQ(parsed.size(), 3);
    ASSERT_EQ(parsed[0].size(), x_values.size());
    ASSERT_EQ(parsed[1].size(), y_values.size());
    ASSERT_EQ(parsed[2].size(), theta_values.size());

    for (int i = 0; i < x_values.size(); i++) {
        EXPECT_NEAR(parsed[0][i], x_values[i],     1.0 / 4096.0);
        EXPECT_NEAR(parsed[1][i], y_values[i],     1.0 / 4096.0);
        EXPECT_NEAR(parsed[2][i], theta_values[i], 1.0 / 4096.0);
    }
}

TEST_F(TESTREAD, testMissingFile) {

    std::string missing_filename_ = "this_doesnt_exist!";
    EXPECT_THROW(ghost_util::readPathFromFile(missing_filename_), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
