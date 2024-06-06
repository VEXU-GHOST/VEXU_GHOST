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

#include "ghost_util/search_util.hpp"
#include "gtest/gtest.h"

using namespace ghost_util;

TEST(TestSearchUtil, testGetIndex) {
  std::vector<double> vector{1.0, 2.0, 3.0, 5.0, 6.0};
  double v1 = 1.5;
  double v2 = 5.1;
  double v3 = 5.0;
  double v4 = -1.0;
  double v5 = 7.0;

  EXPECT_EQ(getInsertionIndexFromSortedVector(v1, vector), 1);
  EXPECT_EQ(getInsertionIndexFromSortedVector(v2, vector), 4);
  EXPECT_EQ(getInsertionIndexFromSortedVector(v3, vector), 4);
  EXPECT_EQ(getInsertionIndexFromSortedVector(v4, vector), 0);
  EXPECT_EQ(getInsertionIndexFromSortedVector(v5, vector), 5);
}

TEST(TestSearchUtil, testThrowsOnUnsortedVector) {
  std::vector<double> vector{1.0, 3.0, 2.0};
  double v1 = 1.5;

  EXPECT_THROW(getInsertionIndexFromSortedVector(v1, vector), std::runtime_error);
}

TEST(TestSearchUtil, testInsertSorted1) {
  std::vector<double> vector{1.0, 2.0, 3.0, 5.0, 6.0};
  double v = 1.5;

  std::vector<double> expected_vector{1.0, 1.5, 2.0, 3.0, 5.0, 6.0};
  EXPECT_EQ(insertValueIntoSortedVector(v, vector), 1);
  EXPECT_EQ(vector, expected_vector);
}

TEST(TestSearchUtil, testInsertSorted2) {
  std::vector<double> vector{1.0, 2.0, 3.0, 5.0, 6.0};
  double v = 5.1;

  std::vector<double> expected_vector{1.0, 2.0, 3.0, 5.0, 5.1, 6.0};
  EXPECT_EQ(insertValueIntoSortedVector(v, vector), 4);
  EXPECT_EQ(vector, expected_vector);
}

TEST(TestSearchUtil, testInsertSorted3) {
  std::vector<double> vector{1.0, 2.0, 3.0, 5.0, 6.0};
  double v = 5.0;

  std::vector<double> expected_vector{1.0, 2.0, 3.0, 5.0, 5.0, 6.0};
  EXPECT_EQ(insertValueIntoSortedVector(v, vector), 4);
  EXPECT_EQ(vector, expected_vector);
}

TEST(TestSearchUtil, testInsertSorted4) {
  std::vector<double> vector{1.0, 2.0, 3.0, 5.0, 6.0};
  double v = -1.0;

  std::vector<double> expected_vector{-1.0, 1.0, 2.0, 3.0, 5.0, 6.0};
  EXPECT_EQ(insertValueIntoSortedVector(v, vector), 0);
  EXPECT_EQ(vector, expected_vector);
}

TEST(TestSearchUtil, testInsertSorted5) {
  std::vector<double> vector{1.0, 2.0, 3.0, 5.0, 6.0};
  double v = 7.0;

  std::vector<double> expected_vector{1.0, 2.0, 3.0, 5.0, 6.0, 7.0};
  EXPECT_EQ(insertValueIntoSortedVector(v, vector), 5);
  EXPECT_EQ(vector, expected_vector);
}
