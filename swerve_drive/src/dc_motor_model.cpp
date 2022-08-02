/*
 * Filename: dc_motor_model
 * Created Date: Sunday July 17th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Sunday July 17th 2022 10:31:58 am
 * Modified By: Maxx Wilson
 */

#include "dc_motor_model.hpp"

#include <algorithm>

namespace dc_motor_model
{

    DCMotorModel::DCMotorModel(
        double free_speed,
        double stall_torque,
        double free_current,
        double stall_current,
        double nominal_voltage,
        double max_speed,
        double max_current) : free_speed_(free_speed),
                              stall_torque_(stall_torque),
                              free_current_(free_current),
                              stall_current_(stall_current),
                              nominal_voltage_(nominal_voltage),
                              gear_ratio_(1.0),
                              max_speed_(max_speed),
                              max_current_(max_current)
    {
        curr_voltage_ = 0;
        curr_speed_ = 0;
        curr_current_ = 0;
        curr_torque_ = 0;
    }

    void DCMotorModel::setMotorInput(double voltage_input, double current_speed)
    {
        // Transform to internal motor speed
        current_speed /= gear_ratio_;
        
        // Save motor direction, then treat speed as only positive
        curr_motor_dir_ = (curr_speed_ >= 0) ? 1 : -1;
        cmd_motor_dir_ = (voltage_input >= 0) ? 1 : -1;

        // Clip inputs to bounds
        curr_voltage_ = std::clamp(voltage_input, -1.0, 1.0);
        curr_speed_ = std::clamp(std::abs(current_speed), 0.0, max_speed_);

        // Update torque output and current draw
        updateMotor();
    }

    void DCMotorModel::updateMotor()
    {
        // Calculate ideal current draw (no limit) at nominal voltage
        curr_current_ = std::abs((stall_current_ - curr_speed_ * (stall_current_ - free_current_) / free_speed_));

        // Clamp if true current exceeds max current
        curr_current_ = (curr_voltage_ * curr_current_ > max_current_) ? max_current_ : curr_current_;

        // Set current limited output torque at nominal voltage
        curr_torque_ = stall_torque_ * (1 - (stall_current_ - curr_current_) / (stall_current_ - free_current_));

        // Scale torque and current outputs based on voltage input
        curr_torque_ *= curr_voltage_ * curr_motor_dir_;
        curr_current_ *= curr_voltage_;
    }

} // namespace dc_motor_model