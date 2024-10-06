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

#pragma once

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
