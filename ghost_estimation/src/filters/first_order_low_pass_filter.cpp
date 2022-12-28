#include <math.h>

#include "ghost_estimation/filters/first_order_low_pass_filter.hpp"

namespace ghost_estimation
{

FirstOrderLowPassFilter::FirstOrderLowPassFilter(float tau, float ts)
    : tau_(tau), ts_(ts), y0_(0.0), y1_(y0_)
{
}

float FirstOrderLowPassFilter::updateFilter(float u1)
{
    // Update filter state
    y1_ = y0_;

    // Evaluate Transfer Function
    y0_ = (y1_ - u1 / tau_) / exp(-ts_ / tau_);
    
    return y0_;
}

} // namespace ghost_estimation