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

#include <math.h>

#include "ghost_estimation/filters/first_order_low_pass_filter.hpp"

namespace ghost_estimation
{

FirstOrderLowPassFilter::FirstOrderLowPassFilter(float w0, float ts)
: w0_(w0),
  ts_(ts),
  y0_(0.0),
  y1_(0.0)
{
}

float FirstOrderLowPassFilter::updateFilter(float u1)
{
  // Update filter state
  y1_ = y0_;

  // Evaluate Transfer Function
  y0_ = y1_ * exp(-w0_ * ts_) + (1 - exp(-w0_ * ts_)) * u1;

  return y0_;
}

float FirstOrderLowPassFilter::getCurrentState()
{
  return y0_;
}

} // namespace ghost_estimation
