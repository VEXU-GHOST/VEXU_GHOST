/*
 * Filename: dc_motor_model
 * Created Date: Sunday July 17th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Sunday July 17th 2022 10:44:41 am
 * Modified By: Maxx Wilson
 */

#ifndef DC_MOTOR_MODEL_HPP
#define DC_MOTOR_MODEL_HPP

#include <limits>

namespace dc_motor_model
{

    class DCMotorModel
    {
    public:

        /**
         * @brief Default Constructor for DC Motor Model
         */
        DCMotorModel() : DCMotorModel(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    std::numeric_limits<double>::infinity()) {};

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
            double nominal_voltage) : 
                DCMotorModel(
                    free_speed,
                    stall_torque,
                    free_current,
                    stall_current,
                    nominal_voltage,
                    free_speed,
                    std::numeric_limits<double>::infinity()) {}

        /**
         * @brief Constructor for DC Motor Model with optional speed and current limits
         *
         * @param free_speed        RPM
         * @param stall_torque      N-m
         * @param free_current      Amps
         * @param stall_current     Amps
         * @param nominal_voltage   Volts
         * @param max_speed         RPM
         * @param max_current       Amps
         */
        DCMotorModel(
            double free_speed,
            double stall_torque,
            double free_current,
            double stall_current,
            double nominal_voltage,
            double max_speed,
            double max_current);

        void setGearRatio(double ratio){
            gear_ratio_ = ratio;
        }

        /**
         * @brief Set new motor input voltage and current speed, then update motor state
         *
         * @param voltage_input voltage percentage (0.0 - 1.0)
         * @param current_speed motor speed (RPM
         */
        void setMotorInput(double voltage_input, double current_speed);

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
         * @brief Update motor outputs based on current state
         */
        void updateMotor();

        /**
         * @brief Get current motor speed
         *
         * @return double speed (RPM)
         */
        double getSpeed()
        {
            return curr_motor_dir_*curr_speed_*gear_ratio_;
        }

        /**
         * @brief Get current input voltage
         *
         * @return double voltage (volts)
         */
        double getVoltage()
        {
            return curr_voltage_ * nominal_voltage_;
        }

        /**
         * @brief Get instantaneous current draw
         *
         * @return double current (amps)
         */
        double getCurrentDraw()
        {
            return curr_current_;
        }

        /**
         * @brief Get current motor output torque
         *
         * @return double torque (N-m)
         */
        double getTorqueOutput()
        {
            return curr_torque_/gear_ratio_;
        }

    private:
        // Motor params
        double free_speed_;      // RPM
        double stall_torque_;    // N-m
        double free_current_;    // Amps
        double stall_current_;   // Amps
        double nominal_voltage_; // Volts
        double gear_ratio_;

        // Motor limits
        double max_speed_;   // RPM
        double max_current_; // Amps

        // Current state
        double curr_voltage_; // %
        double curr_speed_;   // RPM
        double curr_torque_;  // N-m
        double curr_current_; // Amps
        int curr_motor_dir_;
        int cmd_motor_dir_;
    };

} // namespace dc_motor_model

#endif