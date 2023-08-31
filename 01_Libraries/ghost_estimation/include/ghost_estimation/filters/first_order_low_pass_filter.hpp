#ifndef GHOST_ESTIMATION__FIRST_ORDER_LOW_PASS_FILTER_HPP
#define GHOST_ESTIMATION__FIRST_ORDER_LOW_PASS_FILTER_HPP

#include <math.h>

namespace ghost_estimation {

/**
 * @brief Implements Discrete First Order Low Pass Filter
 */
class FirstOrderLowPassFilter {
public:
	/**
	 * @brief Constructor for First Order Low Pass Filter Object
	 *
	 * Filter parameters assume filter designed in continuous laplace domain.
	 * Discretization is performed internally using Z-Transform Table.
	 *
	 * See: http://lpsa.swarthmore.edu/LaplaceZTable/LaplaceZFuncTable.html
	 *
	 * Functionality is not changed by frequency units (can be Hz or rad/s) as long as input data matches.
	 *
	 * @param w0   filter cutoff frequency
	 * @param ts    timestep in seconds
	 */
	FirstOrderLowPassFilter(float w0, float ts);

	/**
	 * @brief Called at regular interval specific by timestep with new raw data input.
	 * Performs Discrete time second order low pass filter.
	 * See: http://lpsa.swarthmore.edu/LaplaceZTable/LaplaceZFuncTable.html
	 *
	 * Derivation:
	 * G(s) = 1 / (s / a + 1) = a/(s + a)
	 *
	 * <Apply Zero-Order Hold Discretization (ZOH)>
	 * Gzoh(s)*G(s) = (1-exp(-Ts)) * a/(s*(s + a))
	 *
	 * G(z) = Z[Gzoh(s)*G(s)] = Z[(a/(s*(s + a))] - Z[(1-exp(-Ts)) * a/(s*(s + a))]
	 *      = (z-1)/z * Z[(a/(s*(s + a))]
	 *      = (z-1)/z * (1-exp(-aT))*z/((z-1)*(z-exp(-aT))) = (1-exp(-aT)) / (z-exp(-aT))
	 *
	 * Y(Z) * (z-exp(-aT)) = U(Z) * (1-exp(-aT))
	 * y[k+1] - y[k]*exp(-aT) = (1-exp(-aT))*u[k]
	 * y[k] = y[k-1] * exp(-w0 * ts) + (1 - exp(-w0 * ts)) * u[k-1]
	 *
	 * @param raw_sensor_input
	 * @return float filtered_output
	 */
	float updateFilter(float u1);

	/**
	 * @brief Returns last filter output
	 *
	 * @return float
	 */
	float getCurrentState();

private:
	// Filter Parameters
	float w0_;
	float ts_;

	// State Variables
	float y0_;
	float y1_;
};

} // namespace ghost_estimation

#endif // GHOST_ESTIMATION__SECOND_ORDER_LOW_PASS_FILTER_HPP