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
#include <iostream>

namespace ghost_control
{

namespace v5_current_limiting
{

double convertMotorToBatteryCurrent(double current)
{
  return exp(((current / 1000) + 4.324) / 0.925);
}

double convertBatteryToMotorCurrent(double current)
{
  return (0.925 * log(current) - 4.324) * 1000;
}

std::vector<double> calculateAllCurrentLimits(std::vector<double> active_current_limits_ma, int num_motors)
{
  double default_current_limit_milliamps = convertBatteryToMotorCurrent(MAX_CURRENT / num_motors);
  default_current_limit_milliamps = ghost_util::clamp(default_current_limit_milliamps, 0.0, 2500.0);

  // Sum Current Limits
  double limited_sum = 0.0;
  int num_limited = 0;
  for (const auto & lim_milliamps : active_current_limits_ma) {
    if (lim_milliamps < default_current_limit_milliamps) {
      limited_sum += convertMotorToBatteryCurrent(lim_milliamps);
      num_limited++;
    }
  }

  int num_unlimited = num_motors - num_limited;
  double battery_current_per_motor = (MAX_CURRENT - limited_sum) / num_unlimited;

  double adjusted_current_limit = convertBatteryToMotorCurrent(battery_current_per_motor);
  adjusted_current_limit = ghost_util::clamp(adjusted_current_limit, 0.0, 2500.0);

  std::vector<double> final_current_limits(num_motors, adjusted_current_limit);

  for (int i = 0; i < active_current_limits_ma.size(); i++) {
    if (active_current_limits_ma[i] < adjusted_current_limit) {
      final_current_limits[i] = active_current_limits_ma[i];
    }
  }
  return final_current_limits;
}

double getRemainingCurrentDistributed(std::vector<double> active_current_limits_ma, int num_motors)
{
  // Calculate the naive default limit
  double default_current_limit_milliamps = convertBatteryToMotorCurrent(MAX_CURRENT / num_motors);
  default_current_limit_milliamps = ghost_util::clamp(default_current_limit_milliamps, 0.0, 2500.0);

  int num_distributed = num_motors - active_current_limits_ma.size();

  // Gather necessary metrics on the active current limits
  int num_over = 0;
  double max_over_current = 0.0;
  std::vector<double> under_currents;
  for (const auto & lim : active_current_limits_ma) {
    if (lim < default_current_limit_milliamps) {
      under_currents.push_back(lim);
    } else {
      num_over++;
      max_over_current = std::max(max_over_current, lim);
    }
  }

  // Get the battery current per unregulated motor
  // Note: motors are divided based on whether they exceed the naive threshold or not, so if the threshold is 1500, then there is
  // no difference at the motor if we were to request 2000mA or 2500mA. This is why we take the max instead of an average.
  auto battery_current_per_motor = convertMotorToBatteryCurrent(max_over_current);

  // Invert the distributing operation and determine what all the limited motors need to add up to.
  auto lim_current_sum = MAX_CURRENT - battery_current_per_motor * num_over;

  // Remove user-specified under currents individually after transforming through exponential to battery.
  for (const auto & current : under_currents) {
    lim_current_sum -= convertMotorToBatteryCurrent(current);
  }

  // Finally, we have the sum of all the unspecified limited motors.
  // We can divide to get the individual currents at the battery, and then convert to the requested motor limits.
  lim_current_sum /= num_distributed;
  auto distributed_current_limit = convertBatteryToMotorCurrent(lim_current_sum);

  // Clamp to max current
  distributed_current_limit = std::min(2500.0, distributed_current_limit);

  // This is a patch for the corner case where we call this without any over current motors (which is non-sensical).
  // Regardless, in that case, we want to return the correct answer, which ignores the default current limiting below.
  if(num_over == 0){
    return distributed_current_limit;
  }

  // For the individual limits to be valid, they must be below the nominal limit, otherwise they won't trigger the algorithm.
  // We drop 1mA to satisfy the strict inequality.
  distributed_current_limit = std::min(default_current_limit_milliamps-1, distributed_current_limit);

  // If we ended up with 1mA below 2500.0, we aren't actually doing any current limiting, so just return full power. 
  if(std::fabs(distributed_current_limit - 2499.0) < std::numeric_limits<float>::epsilon()){
    return 2500.0;
  }

  return distributed_current_limit;
}

} // namespace v5_current_limiting
} // namespace ghost_control
