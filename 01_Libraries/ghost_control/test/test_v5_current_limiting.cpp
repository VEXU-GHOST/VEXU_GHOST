/*
 *   Copyright (c) 2025 Maxx Wilson
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

#include <ghost_control/models/v5_current_limiting.hpp>

#include "gtest/gtest.h"

using namespace ghost_control::v5_current_limiting;

TEST(testV5CurrentLimiting, testLessThanEightMotorsAreFullPower) {
  std::vector<double> active_current_limits;
  for (int i = 1; i < 9; i++) {
    auto result = calculateAllCurrentLimits(active_current_limits, i);
    EXPECT_EQ(result.size(), i);
    for (const auto & lim : result) {
      EXPECT_FLOAT_EQ(lim, 2500.0);
    }
  }
}

TEST(testV5CurrentLimiting, testMoreThanEightMotorsAreLimitedEqually) {
  std::vector<double> active_current_limits;
  for (int i = 9; i < 21; i++) {
    auto result = calculateAllCurrentLimits(active_current_limits, i);
    EXPECT_EQ(result.size(), i);
    for (const auto & lim : result) {
      EXPECT_FLOAT_EQ(lim, convertBatteryToMotorCurrent(MAX_CURRENT / i));
    }
  }
}

TEST(testV5CurrentLimiting, testMotorsLimitedToZero) {
  // Sweep all possible numbers of motors
  for (int num_motors = 1; num_motors < 21; num_motors++) {
    // For each number of motors, limit a subset
    for (int num_limited = 1; num_limited < num_motors; num_limited++) {
      double num_unlimited = num_motors - num_limited;

      std::vector<double> active_current_limits(num_limited, 0.0);
      auto result = calculateAllCurrentLimits(active_current_limits, num_motors);

      for (int k = 0; k < result.size(); k++) {
        if (k < num_limited) {
          // Limited Subset are zero
          EXPECT_EQ(result[k], 0.0);
        } else {
          // So this part is odd. A motor that is plugged in but set to zero still lowers the overall current capacity because of the log/exp transform used...
          double limited_battery_current = num_limited * convertMotorToBatteryCurrent(0.0);
          double adjusted_current_limit = convertBatteryToMotorCurrent((MAX_CURRENT - limited_battery_current) / (num_unlimited));

          // Cap at 2.5A
          adjusted_current_limit = std::min(2500.0, adjusted_current_limit);
          EXPECT_FLOAT_EQ(result[k], adjusted_current_limit);
        }
      }
    }
  }
}

TEST(testV5CurrentLimiting, testSubsetOfMotorsLimitedToNonZeroValue) {
  int num_motors = 16;
  int num_limited = 2;
  double current_limit = 250.0;
  std::vector<double> active_current_limits(num_limited, current_limit);
  auto result = calculateAllCurrentLimits(active_current_limits, num_motors);

  for (int i = 0; i < num_motors; i++) {
    if (i < num_limited) {
      EXPECT_FLOAT_EQ(result[i], current_limit);
    } else {
      EXPECT_FLOAT_EQ(result[i], 1962.257);
    }
  }
}

TEST(testV5CurrentLimiting, testCurrentLimitsOverRegulatedAreIgnored) {
  int num_motors = 16;
  int num_limited = 2;
  double current_limit = 1870;
  std::vector<double> active_current_limits(num_limited, current_limit);
  auto result = calculateAllCurrentLimits(active_current_limits, num_motors);

  for (int i = 0; i < num_motors; i++) {
    EXPECT_FLOAT_EQ(result[i], 1859.266);
  }
}


TEST(testV5CurrentLimiting, testGetRemainingCurrentLimitsUnthrottled) {
  int num_motors = 16;
  std::vector<double> active_current_limits{2500.0, 2500.0};
  EXPECT_FLOAT_EQ(1716.8187, getRemainingCurrentLimitsUnthrottled(active_current_limits, num_motors));
}

TEST(testV5CurrentLimiting, testGetRemainingCurrentLimitsUnthrottledUneven) {
  int num_motors = 20;
  std::vector<double> active_current_limits{1450.0, 118.0, 2485.0};
  EXPECT_FLOAT_EQ(1627.9312, getRemainingCurrentLimitsUnthrottled(active_current_limits, num_motors));
}

TEST(testV5CurrentLimiting, testBelowCurrentLimits) {
  int num_motors = 6;
  std::vector<double> active_current_limits{2500.0};
  EXPECT_FLOAT_EQ(2500.0, getRemainingCurrentLimitsUnthrottled(active_current_limits, num_motors));
}

TEST(testV5CurrentLimiting, testAtCurrentLimits) {
  int num_motors = 8;
  std::vector<double> active_current_limits{2500.0};
  EXPECT_FLOAT_EQ(2500.0, getRemainingCurrentLimitsUnthrottled(active_current_limits, num_motors));
}

TEST(testV5CurrentLimiting, testAboveCurrentLimitZeroed) {
  int num_motors = 9;
  std::vector<double> active_current_limits{0.0};
  EXPECT_FLOAT_EQ(2492.6480, getRemainingCurrentLimitsUnthrottled(active_current_limits, num_motors));
}