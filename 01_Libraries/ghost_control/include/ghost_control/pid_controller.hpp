/*
 *   Copyright (c) 2025 Maxx Wilson
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

#include <ghost_util/math_util.hpp>

namespace ghost_control
{

struct PIDGains
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral_limit = 0.0;
};

/**
 * @brief A simple PID controller with integral windup protection and error sign reset.
 *
 * This class calculates control commands based on a proportional-integral-derivative (PID) algorithm.
 */
class PIDController
{
public:
  /**
   * @brief Constructs a PID controller with given gains and timestep.
   * @param gains Struct containing the PID gains.
   * @param dt Time step between control loop updates (in seconds).
   */
  PIDController(const PIDGains & gains, double dt = 0.01);

  /**
   * @brief Resets the integral accumulator and previous error state.
   */
  void reset();

  /**
   * @brief Computes a control command based on error and error derivative.
   *
   * If the error changes sign, the integral term is reset to prevent windup.
   *
   * @param error Current error between target and measured value.
   * @param error_deriv Derivative of the error.
   * @return Control output.
   */
  double calculateCommand(double error, double error_deriv, double additional_terms = 0.0);

private:
  PIDGains gains_;
  double integral_sum_;
  double last_error_;
  double dt_;
};

} // namespace ghost_control
