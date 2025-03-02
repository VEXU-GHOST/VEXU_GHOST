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

using namespace ghost_control::v5_current_limiting;
#include <iostream>

int main(int argc, char * argv[])
{
  int num_motors = 16;
  std::vector<double> active_current_limits{2500.0, 2000.0, 2500.0, 0.0, 120.0};
  auto throttled = getRemainingCurrentDistributed(active_current_limits, num_motors);
  std::cout << "Remaining Unthrottled: " << throttled << std::endl;

  std::vector<double> custom_current_limits;
  custom_current_limits.insert(custom_current_limits.end(), active_current_limits.begin(), active_current_limits.end());
  for (int i = 0; i < num_motors - active_current_limits.size(); i++) {
    custom_current_limits.push_back(throttled);
  }
  auto result = calculateAllCurrentLimits(custom_current_limits, num_motors);


  double sum = 0.0;
  for (const auto & lim : result) {
    sum += lim;
    std::cout << lim << std::endl;
  }
  std::cout << "Sum: " << sum << std::endl;
}
