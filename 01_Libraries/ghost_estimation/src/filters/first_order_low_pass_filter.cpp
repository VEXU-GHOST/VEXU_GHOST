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
