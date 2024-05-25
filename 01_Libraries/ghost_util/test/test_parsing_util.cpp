/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include <iostream>
#include "ghost_util/parsing_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;

class TestParsingUtil : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }
};

TEST_F(TestParsingUtil, parseIntVector) {
  std::string test = "1 2 3 4 5";
  std::vector<int> output = getVectorFromString<int>(test, ' ');
  std::vector<int> solution{1, 2, 3, 4, 5};

  EXPECT_EQ(output, solution);
}


TEST_F(TestParsingUtil, parseFloatVector) {
  std::string test = "1.1 2.3 3.5";
  std::vector<float> output = getVectorFromString<float>(test, ' ');
  std::vector<float> solution{1.1, 2.3, 3.5};

  EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseDoubleVector) {
  std::string test = "1.1 2.3 3.5";
  std::vector<double> output = getVectorFromString<double>(test, ' ');
  std::vector<double> solution{1.1, 2.3, 3.5};

  EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseFloatVectorFromInt) {
  std::string test = "1 2 3";
  std::vector<float> output = getVectorFromString<float>(test, ' ');
  std::vector<float> solution{1.0, 2.0, 3.0};

  EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseDoubleVectorFromInt) {
  std::string test = "1 2 3";
  std::vector<double> output = getVectorFromString<double>(test, ' ');
  std::vector<double> solution{1.0, 2.0, 3.0};

  EXPECT_EQ(output, solution);
}

TEST_F(TestParsingUtil, parseStringVector) {
  std::string test = "1.1 2.3 test";
  std::vector<std::string> output = getVectorFromString<std::string>(test, ' ');
  std::vector<std::string> solution{"1.1", "2.3", "test"};

  EXPECT_EQ(output, solution);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
