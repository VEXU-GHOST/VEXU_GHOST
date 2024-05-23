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

#include "ghost_util/math_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;

class TestMathUtil : public ::testing::Test
{
public:
  TestMathUtil()
  {
  }
};

TEST_F(TestMathUtil, testInterpolate) {
  using ghost_util::linearInterpolate;

  std::vector<double> x_data = {1.0, 2.0};
  std::vector<double> y_data = {0.0};

  EXPECT_THROW(linearInterpolate(x_data, y_data, 2.0), std::runtime_error);

  x_data = {1.0, 2.0, 3.0, 4.0};
  y_data = {2.0, 0.0, 1.0, 6.0};

  EXPECT_EQ(linearInterpolate(x_data, y_data, 2.0), 0.0);       // exact match

  EXPECT_EQ(linearInterpolate(x_data, y_data, 0.75), 2.5);        // (-inf,1]
  EXPECT_EQ(linearInterpolate(x_data, y_data, 1.25), 1.5);        // [1,2]
  EXPECT_EQ(linearInterpolate(x_data, y_data, 2.5), 0.5);         // [2,3]
  EXPECT_EQ(linearInterpolate(x_data, y_data, 3.25), 2.25);       // [3,4]
  EXPECT_EQ(linearInterpolate(x_data, y_data, 4.75), 9.75);       // [4,+inf]
}

TEST_F(TestMathUtil, testClampedInterpolate) {
  using ghost_util::clampedLinearInterpolate;

  std::vector<double> x_data = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> y_data = {2.0, 0.0, 1.0, 6.0};

  EXPECT_EQ(clampedLinearInterpolate(x_data, y_data, 0.0), 2.0);       // (-inf,1]
  EXPECT_EQ(clampedLinearInterpolate(x_data, y_data, 5.0), 6.0);       // [4,+inf)
}

TEST_F(TestMathUtil, testSlewRate) {
  using ghost_util::slewRate;

  EXPECT_DOUBLE_EQ(slewRate(1, 1, 2), 1);

  // sign of limit
  EXPECT_DOUBLE_EQ(slewRate(0, 2, 2), 2.0);
  EXPECT_DOUBLE_EQ(slewRate(0, 2, -2), 2.0);
  EXPECT_DOUBLE_EQ(slewRate(0, -2, -2), -2.0);
  EXPECT_DOUBLE_EQ(slewRate(0, -2, 2), -2.0);

  // random values in directions
  EXPECT_DOUBLE_EQ(slewRate(-1.6, 1, .6), -1);
  EXPECT_DOUBLE_EQ(slewRate(-1.6, 0, .6), -1);
  EXPECT_DOUBLE_EQ(slewRate(-1.6, 1, .7), -.9);
  EXPECT_DOUBLE_EQ(slewRate(-1.6, 0, .7), -.9);

  // random values
  EXPECT_DOUBLE_EQ(slewRate(0, -2, 1), -1.0);
  EXPECT_DOUBLE_EQ(slewRate(0, 2, 1), 1.0);
  EXPECT_DOUBLE_EQ(slewRate(2, 1, 1), 1.0);
  EXPECT_DOUBLE_EQ(slewRate(2, -1, 1), 1.0);
  EXPECT_DOUBLE_EQ(slewRate(2, -1000, 1), 1.0);
  EXPECT_DOUBLE_EQ(slewRate(-2, -1000, 1), -3.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
