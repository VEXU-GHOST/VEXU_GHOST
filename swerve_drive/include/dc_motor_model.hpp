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
#include <algorithm>

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
                    0.0) {};

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
            double nominal_voltage);

        void setGearRatio(double ratio){
            gear_ratio_ = ratio;
        }

        void setCurrentLimit(double current_lim){
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
        void setMotorSpeedRad(double current_speed){
            setMotorSpeedRPM(current_speed*60/(2*3.14159));
        };

        /**
         * @brief Get current motor speed
         *
         * @return double speed (RPM)
         */
        double getSpeedRPM()
        {
            return curr_speed_*gear_ratio_;
        }

        /**
         * @brief Get current motor speed
         *
         * @return double speed (rad/s)
         */
        double getSpeedRad()
        {
            return curr_speed_*gear_ratio_*(2*3.14159)/60;
        }

        /**
         * @brief Get current input voltage
         *
         * @return double voltage (volts)
         */
        double getVoltage()
        {
            return curr_voltage_percent_ * nominal_voltage_;
        }

        /**
         * @brief Get instantaneous current draw
         *
         * @return double current (amps)
         */
        double getMotorCurrent()
        {
            return std::abs(curr_current_);
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

        void updateMotor();

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
        double curr_voltage_percent_; // %
        double curr_speed_;   // RPM
        double curr_torque_;  // N-m
        double curr_current_; // Amps
        int curr_motor_dir_;
        int cmd_motor_dir_;
    };

} // namespace dc_motor_model

#endif