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

#include "ghost_control/motor_controller.hpp"

#include "ghost_util/math_util.hpp"

namespace ghost_control
{

MotorController::MotorController(
  const MotorController::Config & controller_config,
  const SecondOrderLowPassFilter::Config & filter_config,
  const DCMotorModel::Config & model_config)
: controller_config_(controller_config),
  filter_config_(filter_config),
  model_config_(model_config),
  velocity_filter_(filter_config),
  motor_model_(model_config_)
{
}

float MotorController::updateMotor(float position, float velocity)
{
  // Update Low Pass Filter with velocity measurement
  auto curr_vel_rpm = velocity_filter_.updateFilter(velocity);

  // Update DC Motor Model
  motor_model_.setMotorSpeedRPM(curr_vel_rpm);

  float max_voltage_mv = model_config_.nominal_voltage * 1000;

  // Calculate control inputs
  float position_feedback = (position_active_) ? (des_pos_encoder_ - position) *
    controller_config_.pos_gain : 0.0;
  float velocity_feedforward = (velocity_active_) ? motor_model_.getVoltageFromVelocityMillivolts(
    des_vel_rpm_) * controller_config_.ff_vel_gain : 0.0;
  float velocity_feedback = (velocity_active_) ? (des_vel_rpm_ - curr_vel_rpm) *
    controller_config_.vel_gain : 0.0;
  float torque_feedforward = (torque_active_) ? motor_model_.getVoltageFromTorqueMillivolts(
    des_torque_nm_) * controller_config_.ff_torque_gain : 0.0;
  float voltage_feedforward = (voltage_active_) ? des_voltage_norm_ * max_voltage_mv : 0.0;

  // Set voltage command from sum of controller terms
  cmd_voltage_mv_ = voltage_feedforward + torque_feedforward + velocity_feedforward +
    velocity_feedback + position_feedback;

  // Clamp actuator limits
  cmd_voltage_mv_ = ghost_util::clamp(cmd_voltage_mv_, -max_voltage_mv, max_voltage_mv);

  // Motor control mode must be constantly refreshed
  if (loops_since_last_cmd_ >= controller_config_.cmd_duration) {
    velocity_active_ = false;
    torque_active_ = false;
    voltage_active_ = false;
    position_active_ = false;
  }

  loops_since_last_cmd_++;
  return cmd_voltage_mv_;
}

} // namespace ghost_control
