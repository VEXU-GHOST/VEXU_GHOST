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
#include <algorithm>
#include <limits>

namespace ghost_control
{

class DCMotorModel
{
public:
  struct Config
  {
    // This configures an 11W V5 Motor.
    float free_speed{120};
    float stall_torque{3.6};
    float free_current{0.14};
    float stall_current{4.25};
    float nominal_voltage{12};
    float gear_ratio{1.0};

    bool operator==(const Config & rhs) const
    {
      return (free_speed == rhs.free_speed) && (stall_torque == rhs.stall_torque) &&
             (free_current == rhs.free_current) &&
             (stall_current == rhs.stall_current) && (nominal_voltage == rhs.nominal_voltage) &&
             (gear_ratio == rhs.gear_ratio);
    }
  };

  /**
   * @brief Constructs DC Motor Model with motor params
   *
   * @param free_speed        RPM
   * @param stall_torque      N-m
   * @param free_current      Amps
   * @param stall_current     Amps
   * @param nominal_voltage   Volts
   */
  DCMotorModel(
    double free_speed,
    double stall_torque,
    double free_current,
    double stall_current,
    double nominal_voltage,
    double gear_ratio = 1.0);

  DCMotorModel(Config config);

  void setGearRatio(double ratio)
  {
    gear_ratio_ = ratio;
  }

  void setCurrentLimit(double current_lim)
  {
    max_current_ = std::abs(current_lim);
  }

  /**
   * @brief Set new motor input effort
   *
   * @param voltage_input voltage percentage (0.0 - 1.0)
   */
  void setMotorEffort(double voltage_percent);

  /**
   * @brief Set maximum motor current
   *
   * @param max_current Amps
   */
  void setMaxCurrent(double max_current)
  {
    max_current_ = max_current;
    updateMotor();
  }

  /**
   * @brief Set Current Motor Speed in RPM
   */
  void setMotorSpeedRPM(double current_speed);

  /**
   * @brief Set Current Motor Speed
   */
  void setMotorSpeedRad(double current_speed)
  {
    setMotorSpeedRPM(current_speed * 60 / (2 * 3.14159));
  }

  /**
   * @brief Get current motor speed
   *
   * @return double speed (RPM)
   */
  double getSpeedRPM() const
  {
    return curr_speed_ * gear_ratio_;
  }

  /**
   * @brief Get current motor speed
   *
   * @return double speed (rad/s)
   */
  double getSpeedRad() const
  {
    return curr_speed_ * gear_ratio_ * (2 * 3.14159) / 60;
  }

  /**
   * @brief Get current input voltage
   *
   * @return double voltage (volts)
   */
  double getVoltage() const
  {
    return curr_voltage_percent_ * nominal_voltage_;
  }

  /**
   * @brief Get instantaneous current draw
   *
   * @return double current (amps)
   */
  double getMotorCurrent() const
  {
    return curr_current_;
  }

  /**
   * @brief Get current motor output torque
   *
   * @return double torque (N-m)
   */
  double getTorqueOutput() const
  {
    return curr_torque_ / gear_ratio_;
  }

  /**
   * @brief Solves for feed forward voltage command given desired torque
   *
   * @param torque_desired
   * @return double voltage_cmd
   */
  double getVoltageFromTorqueMillivolts(double torque_desired) const;

  /**
   * @brief Solves for feed forward voltage command given desired velocity
   *
   * @param velocity_desired
   * @return double voltage_cmd
   */
  double getVoltageFromVelocityMillivolts(double velocity_desired) const;

private:
  void updateMotor();

  // Motor params
  double free_speed_;            // RPM
  double stall_torque_;          // N-m
  double free_current_;          // Amps
  double stall_current_;         // Amps
  double nominal_voltage_;       // Volts
  double gear_ratio_;

  // Motor limits
  double max_speed_;         // RPM
  double max_current_;       // Amps

  // Current state
  double curr_voltage_percent_;       // %
  double curr_speed_;         // RPM
  double curr_torque_;        // N-m
  double curr_current_;       // Amps
  int curr_motor_dir_;
  int cmd_motor_dir_;
};

} // namespace ghost_control
