#ifndef GHOST_ESTIMATION__SECOND_ORDER_LOW_PASS_FILTER_HPP
#define GHOST_ESTIMATION__SECOND_ORDER_LOW_PASS_FILTER_HPP

#include <math.h>

namespace ghost_estimation
{

    /**
     * @brief Implements Discrete Second Order Low Pass Filter
     */
    class SecondOrderLowPassFilter
    {
    public:
        /**
         * @brief Constructor for Second Order Low Pass Filter Object
         * 
         * Filter parameters assume filter designed in continuous laplace domain.
         * Discretization is performed internally using Z-Transform Table.
         *
         * See: http://lpsa.swarthmore.edu/LaplaceZTable/LaplaceZFuncTable.html
         * 
         * Functionality is not changed by frequency units (can be Hz or rad/s) as long as input data matches.
         * 
         * @param w0    natural frequency (or break/corner frequency)
         * @param zeta  damping factor (between 0.0 and 1.0)
         * @param ts    timestep in seconds
         */
        SecondOrderLowPassFilter(float w0, float zeta, float ts);
        /**
         * @brief Called at regular interval specific by timestep with new raw data input.
         * Performs Discrete time second order low pass filter.
         * See: http://lpsa.swarthmore.edu/LaplaceZTable/LaplaceZFuncTable.html
         * 
         * Derivation:
         * G(s) = w0^2 / (s^2 + 2*zeta*w0*s + w0^2)
         * 
         * <Apply Zero-Order Hold Discretization (ZOH)>
         * Gzoh(s)*G(s) = (1-exp(-Ts)) * w0^2 / (s^2 + 2*zeta*w0*s + w0^2)
         * 
         * G(z) = Z[(1-exp(-Ts)) * w0^2 / (s^2 + 2*zeta*w0*s + w0^2)]
         *      = (z-1)/z * Z[w0^2 / (s*(s^2 + 2*zeta*w0*s + w0^2))]
         *      = 1 - (z-1)/z * (w0/wd) * (Z^2 * wd/w0 + Z * exp^(-zeta*w0*ts) * sin(wd - acos(zeta))) / (Z^2 - Z * 2*exp(-zeta*w0*ts)*cos(wd*ts) + exp(-2*zeta*wo*ts))
         * Y/U  = 1 - (z-1) * (w0/wd) * (n2 * Z^2 + n1 * Z) / (z*(d2 * Z^2 + d1 * Z + d0))
         * Y/U  = (z*(d2 * Z^2 + d1 * Z + d0)) / (z*(d2 * Z^2 + d1 * Z + d0)) - (z-1) * (w0/wd) * (n2 * Z^2 + n1 * Z) / (z*(d2 * Z^2 + d1 * Z + d0))
         * 
         * Y * (d2 * Z^2 + d1 * Z + d0) = U * ((d2 * Z^2 + d1 * Z + d0) - (z-1) * (w0/wd) * (n2 * Z + n1))
         * d2 * y[k+2] + d1 * y[k+1] + d0 * y[k] = d2 * u[k+2] + d1 * u[k+1] + d0 * u[k] - (w0/wd) * (n2 * u[k+2] + n1 * u[k+1]) + (w0/wd) * (n2 * u[k+1] + n1 * u[k])
         *                                       = (d2 - n2*w0/wd) * u[k+2] + (d1 + w0/wd * (n2 - n1)) * u[k+1] + (d0 + n1 * w0 / wd) * u[k]
         * d2 * y[k] + d1 * y[k-1] + d0 * y[k-2] = (d2 - n2*w0/wd) * u[k] + (d1 + w0/wd * (n2 - n1)) * u[k-1] + (d0 + n1 * w0 / wd) * u[k-2])
         * y[k] = ((d2 - n2*w0/wd) * u[k] + (d1 + w0/wd * (n2 - n1)) * u[k-1] + (d0 + n1 * w0 / wd) * u[k-2]) - d1 * y[k-1] - d0 * y[k-2]) / d2
         * 
         * @param input
         * @return float filtered_output
         */
        float updateFilter(float u1);

    private:
        // Filter Parameters
        float w0_;
        float zeta_;
        float ts_;
        
        float wd_;
        float phi_;

        // Transfer Function coefficients
        float n2_coeff_;
        float n1_coeff_;
        float d2_coeff_;
        float d1_coeff_;
        float d0_coeff_;

        // Output State Variables
        float y0_;
        float y1_;
        float y2_;

        // Input State Variables
        float u0_;
        float u1_;
        float u2_;
    };

} // namespace ghost_estimation

#endif // GHOST_ESTIMATION__SECOND_ORDER_LOW_PASS_FILTER_HPP