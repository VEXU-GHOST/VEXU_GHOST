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
#include <ghost_control/pid_controller.hpp>

namespace ghost_control
{

PIDController::PIDController(const PIDConfig & config, double dt)
: config_(config), dt_(dt)
{
  reset();
}

void PIDController::reset()
{
  integral_sum_ = 0.0;
  last_error_ = 0.0;
}

double PIDController::calculateCommand(double error, double error_deriv, double additional_terms)
{
  // Check for integral reset
  if (last_error_ * error < 0) {
    integral_sum_ = 0.0;
  }
  last_error_ = error;

  // Calculate Integral Component
  double integral_component = 0.0;
  if (std::fabs(error) <= config_.integral_activation_bound) {
    integral_sum_ += error * dt_;
    integral_component = ghost_util::clamp(config_.ki * integral_sum_, -config_.integral_limit, config_.integral_limit);
  } else {
    integral_sum_ = 0.0;  // Optional: clear it when outside zone
  }

  return config_.kp * error + integral_component + config_.kd * error_deriv + additional_terms;
}

} // namespace ghost_control
