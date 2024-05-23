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

#include "ghost_control/models/dc_motor_model.hpp"

namespace ghost_control
{

DCMotorModel::DCMotorModel(
  double free_speed,
  double stall_torque,
  double free_current,
  double stall_current,
  double nominal_voltage,
  double gear_ratio)
: free_speed_(free_speed),
  stall_torque_(stall_torque),
  free_current_(free_current),
  stall_current_(stall_current),
  nominal_voltage_(nominal_voltage),
  gear_ratio_(gear_ratio),
  max_current_(stall_current)
{
  curr_voltage_percent_ = 0;
  curr_speed_ = 0;
  curr_current_ = 0;
  curr_torque_ = 0;
}

DCMotorModel::DCMotorModel(Config config)
: DCMotorModel(config.free_speed,
    config.stall_torque,
    config.free_current,
    config.stall_current,
    config.nominal_voltage,
    config.gear_ratio)
{
}


void DCMotorModel::setMotorEffort(double voltage_percent)
{
  // Clip inputs to bounds
  curr_voltage_percent_ = std::clamp(voltage_percent, -1.0, 1.0);
  cmd_motor_dir_ = (curr_voltage_percent_ > 0) ? 1 : -1;

  // Update torque output and current draw
  updateMotor();
}

void DCMotorModel::setMotorSpeedRPM(double current_speed)
{
  curr_speed_ = current_speed / gear_ratio_;
  updateMotor();
}

double DCMotorModel::getVoltageFromTorqueMillivolts(double torque_desired) const
{
  return (torque_desired / stall_torque_ + curr_speed_ / free_speed_) * nominal_voltage_ * 1000;
}

double DCMotorModel::getVoltageFromVelocityMillivolts(double velocity_desired) const
{
  return (velocity_desired / (free_speed_ * gear_ratio_)) * nominal_voltage_ * 1000;
}

void DCMotorModel::updateMotor()
{
  // Calculate current motor torque
  curr_torque_ = (stall_torque_) * curr_voltage_percent_ - (stall_torque_ / free_speed_) *
    curr_speed_;

  // Calculate ideal current draw (no limit)
  curr_current_ = (stall_current_) * curr_voltage_percent_ -
    ((stall_current_ - free_current_) / free_speed_) * curr_speed_;

  // // Clamp if true current exceeds max current
  // curr_current_ = (curr_current_ > max_current_) ? max_current_ : curr_current_;
}

} // namespace ghost_control
