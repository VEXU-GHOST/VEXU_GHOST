#ifndef GHOST_ESTIMATION__FIRST_ORDER_LOW_PASS_FILTER_HPP
#define GHOST_ESTIMATION__FIRST_ORDER_LOW_PASS_FILTER_HPP

#include <math.h>

namespace ghost_estimation
{

    /**
     * @brief Implements Discrete First Order Low Pass Filter
     */
    class FirstOrderLowPassFilter
    {
    public:
        /**
         * @brief Constructor for First Order Low Pass Filter Object
         * 
         * Filter parameters assume filter designed in continuous laplace domain.
         * Discretization is performed internally using Z-Transform Table.
         *
         * See: http://lpsa.swarthmore.edu/LaplaceZTable/LaplaceZFuncTable.html
         * 
         * @param tau   filter time constant
         * @param ts    timestep in seconds
         */
        FirstOrderLowPassFilter(float tau, float ts);
        
        /**
         * @brief Called at regular interval specific by timestep with new raw data input.
         * Performs Discrete time second order low pass filter.
         * See: http://lpsa.swarthmore.edu/LaplaceZTable/LaplaceZFuncTable.html
         * 
         * Derivation:
         * Y/U = 1 / (tau * s + 1)
         *     = (1 / tau) * 1 / (s + 1 / tau)
         *     = (1 / tau) * Z / (Z - exp(-T / tau))
         * 
         * Y * (Z - exp(-T / tau)) = Z * U / tau
         * y[k-1] - y[k] * exp(-T / tau) = u[k-1] / tau
         * y[k] = (y[k-1] - u[k-1] / tau) / exp(-T / tau)
         * 
         * @param raw_sensor_input
         * @return float filtered_output
         */
        float updateFilter(float u1);

    private:
        // Filter Parameters
        float tau_;
        float ts_;

        // Transfer Function coefficients
        float leading_coeff_;
        float nz1_coeff_;
        float dz1_coeff_;
        float dz0_coeff_;

        // State Variables
        float y0_;
        float y1_;
    };

} // namespace ghost_estimation

#endif // GHOST_ESTIMATION__SECOND_ORDER_LOW_PASS_FILTER_HPP