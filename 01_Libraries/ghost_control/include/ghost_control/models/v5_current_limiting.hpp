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

#pragma once
#include <math.h>
#include <vector>
#include <numeric>
#include <ghost_util/math_util.hpp>

namespace ghost_control
{

namespace v5_current_limiting
{
constexpr double MAX_CURRENT = 12800.0;

double convertMotorToBatteryCurrent(double current);

double convertBatteryToMotorCurrent(double current);

/**
 * @brief This replicates the current limiting logic implemented in VEX OS, as described on this page: https://wiki.purduesigbots.com/vex-electronics/vex-electronics/motors.
 * 
 * This is a weird black-box function, and is not user friendly at all, so we include other functions in this module to assist.
 * This is intended just to replicate the existing logic upstream.
 * 
 * 
 * Observations/Notes:
 * If there are less than 9 motors plugged in, they all get the full 2500mA.
 * If there are nine or more, the limits decrease.
 * If there are 8 motors, and N additional motors with current limits set to zero, THEY STILL DECREASE, albiet by a very small amount.
 * If there are nine or more motors plugged in, and a subset are limited to a value higher thatn the nominal decreased limit, then they are throttled to the adjusted limit.
 * 
 * @param active_current_limits_ma Vector containing any active current limits
 * @param num_motors Number of motors plugged in to the brain
 * @return std::vector<double> vector of all current limits, where first N are equal to the active current limits passed in (IF the active limits are less than the adjusted current limit)
 */
std::vector<double> calculateAllCurrentLimits(std::vector<double> active_current_limits_ma, int num_motors);

/**
 * @brief Given N motors plugged into the brain, and M current limits in milliAmps, this method returns what value we should limit all other motors to such that VEX OS will not throttle
 * the desired/"active" motors.
 * 
 * Issue:
 *  We have 20 motors plugged in. The following is completely ignored, because no motor can exceed the adjusted current limit without first lowering other motor values.
 *  ```
 *  motor_1.setCurrentLimit(2500.0);
 *  ```
 * 
 * Solution:
 *  ```
 *  motor1.setCurrentLimit(2500.0);
 *  
 *  lim = getRemainingCurrentLimitsUnthrottled(std::vector<double>(2500.0), 20);
 *  motor2.setCurrentLimit(lim);
 *  ...
 *  motorN.setCurrentLimit(lim);
 *  ```
 * 
 * Now all motors are properly throttled such that motor 1 can run at full power
 * 
 * @param active_current_limits_ma Vector containing any active current limits
 * @param num_motors Number of motors plugged in to the brain
 * @return double   Single value which can be set for all other motors to avoid throttling active current limits
 */
double getRemainingCurrentLimitsUnthrottled(std::vector<double> active_current_limits_ma, int num_motors);

} // namespace v5_current_limiting
} // namespace ghost_control
