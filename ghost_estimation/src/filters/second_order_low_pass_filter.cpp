#include <math.h>

#include "ghost_estimation/filters/second_order_low_pass_filter.hpp"

namespace ghost_estimation
{

SecondOrderLowPassFilter::SecondOrderLowPassFilter(float w0, float zeta, float ts)
    : w0_(w0), zeta_(zeta), ts_(ts), y0_(0.0), y1_(0.0), y2_(0.0), u0_(0.0), u1_(0.0), u2_(0.0)
    {
        // Damped Natural Frequency
        wd_ = w0_ * sqrt(1 - zeta_ * zeta_);
        phi_ = acos(zeta);
        
        // Transfer Function Coefficients
        n2_coeff_ = wd_/w0_;
        n1_coeff_ = exp(-zeta_*w0_*ts_) * sin(wd_*ts_ - phi_);
        d2_coeff_ = 1.0;
        d1_coeff_ = -2*exp(-zeta_*w0_*ts_) * cos(wd_ * ts_);
        d0_coeff_ = exp(-2*zeta_*w0_*ts_);

        // Evaluate difference equation coefficients
        u0_coeff_ = (d2_coeff_ - n2_coeff_*w0_/wd_);
        u1_coeff_ = (d1_coeff_ + w0_/wd_ * (n2_coeff_ - n1_coeff_));
        u2_coeff_ = (d0_coeff_ + n1_coeff_ * w0_ / wd_);
        y0_coeff_ = d2_coeff_;
        y1_coeff_ = - d1_coeff_;
        y2_coeff_ = - d0_coeff_;
    }

float SecondOrderLowPassFilter::updateFilter(float u0)
{
    // Update filter states
    u2_ = u1_;
    u1_ = u0_;
    u0_ = u0;

    y2_ = y1_;
    y1_ = y0_;
    y0_ = (u0_coeff_ * u0_ + u1_coeff_ * u1_ + u2_coeff_ * u2_ + y1_coeff_ * y1_ + y2_coeff_ * y2_) / y0_coeff_;

    return y0_;
}

float SecondOrderLowPassFilter::getCurrentState(){
    return y0_;
}

} // namespace ghost_estimation