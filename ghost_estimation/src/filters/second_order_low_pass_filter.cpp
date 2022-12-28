#include <math.h>

#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"

namespace ghost_estimation
{

SecondOrderLowPassFilter::SecondOrderLowPassFilter(float w0, float zeta, float ts)
    : w0_(w0), zeta_(zeta), ts_(ts), y0_(0.0), y1_(0.0), y2_(0.0)
    {
        // Damped Natural Frequency
        wd_ = w0_ * sqrt(1 - zeta_ * zeta_);

        // Transfer Function Coefficients
        n1_coeff_ = (w0_)/(wd_/w0_) * exp(-zeta_*w0_*ts_) * sin(wd_*ts_);
        d1_coeff_ = -2*exp(-zeta_*w0_*ts_)*cos(wd_*ts_);
        d0_coeff_ = exp(-2*zeta_*w0_*ts_);
    }

float SecondOrderLowPassFilter::updateFilter(float u1)
{
    // Update filter states
    y2_ = y1_;
    y1_ = y0_;

    // Evaluate Transfer Function
    y0_ = (n1_coeff_ * u1 - d1_coeff_ * y1_ - y2_) / d0_coeff_;

    return y0_;
}

} // namespace ghost_estimation