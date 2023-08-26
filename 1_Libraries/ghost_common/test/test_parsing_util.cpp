#include "gtest/gtest.h"
#include "ghost_common/util/parsing_util.hpp"
#include <iostream>

using namespace ghost_common;

class TestParsingUtil: public ::testing::Test {
    protected:

    void SetUp() override {
    }
};

TEST_F(TestParsingUtil, parseIntVector){
    std::string test = "1 2 3 4 5";
    std::vector<int> output = getVectorFromString<int>(test, ' ');
    std::vector<int> solution{1, 2, 3, 4, 5};

    EXPECT_EQ(output, solution);
}


TEST_F(TestParsingUtil, parseFloatVector){
    std::string test = "1.1 2.3 3.5";
    std::vector<float> output = getVectorFromString<float>(test, ' ');
    std::vector<float> solution{1.1, 2.3, 3.5};

    EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseDoubleVector){
    std::string test = "1.1 2.3 3.5";
    std::vector<double> output = getVectorFromString<double>(test, ' ');
    std::vector<double> solution{1.1, 2.3, 3.5};

    EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseFloatVectorFromInt){
    std::string test = "1 2 3";
    std::vector<float> output = getVectorFromString<float>(test, ' ');
    std::vector<float> solution{1.0, 2.0, 3.0};

    EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseDoubleVectorFromInt){
    std::string test = "1 2 3";
    std::vector<double> output = getVectorFromString<double>(test, ' ');
    std::vector<double> solution{1.0, 2.0, 3.0};

    EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseStringVector){
    std::string test = "1.1 2.3 test";
    std::vector<std::string> output = getVectorFromString<std::string>(test, ' ');
    std::vector<std::string> solution{"1.1", "2.3", "test"};

    EXPECT_EQ(output, solution);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}