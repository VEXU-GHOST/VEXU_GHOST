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

double getRemainingCurrentLimitsUnthrottled(std::vector<double> active_current_limits_ma, int num_motors)
{
  double limited_sum = 0.0;
  for (const auto & lim : active_current_limits_ma) {
    limited_sum += convertMotorToBatteryCurrent(lim);
  }
  return std::min(2500.0, convertBatteryToMotorCurrent((MAX_CURRENT - limited_sum) / (num_motors - active_current_limits_ma.size())));
}

} // namespace v5_current_limiting
} // namespace ghost_control
