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
         * @param w0    natural frequency (or break/corner frequency) in rad/s
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
         * Y/U = (n1 * Z) / (Z^2 + d1*Z + d0)
         * Y * (Z^2 + d1*Z + d0) = U * (n1 * Z)
         * y[k-2] + d1*y[k-1] + d0 * y[k] = n1 * u[k-1]
         * y[k] = (n1 * u[k-1] - d1*y[k-1] - y[k-2]) / d0
         * 
         * @param input
         * @return float filtered_output
         */
        float updateFilter(float u1);

    private:
        // Filter Parameters
        float w0_;
        float wd_;
        float zeta_;
        float ts_;

        // Transfer Function coefficients
        float n1_coeff_;
        float d1_coeff_;
        float d0_coeff_;

        // State Variables
        float y0_;
        float y1_;
        float y2_;
    };

} // namespace ghost_estimation

#endif // GHOST_ESTIMATION__SECOND_ORDER_LOW_PASS_FILTER_HPP