

#include "ghost_control/models/mechanical_differential.hpp"

namespace ghost_control{

    MechanicalDifferential::MechanicalDifferential(float carrier_ratio, float output_ratio){
        carrier_ratio_ = carrier_ratio;
        output_ratio_ = output_ratio;

        jacobian_ << output_ratio / 2.0, -output_ratio / 2.0, carrier_ratio / 2.0, carrier_ratio / 2.0;
        jacobian_inverse_ << 1/output_ratio, 1/carrier_ratio, -1/output_ratio, 1/carrier_ratio;
        jacobian_transpose_ = jacobian_.transpose();
        jacobian_inverse_transpose_ = jacobian_inverse_.transpose();
    }

    Eigen::Vector2f MechanicalDifferential::getInputTorque(float tau_output, float tau_carrier){
        return jacobian_transpose_ * Eigen::Vector2f(tau_output, tau_carrier);
    }

    Eigen::Vector2f MechanicalDifferential::getOutputTorque(float tau_1, float tau_2){
        return jacobian_inverse_transpose_ * Eigen::Vector2f(tau_1, tau_2);
    }

    Eigen::Vector2f MechanicalDifferential::getInputVelocity(float vel_output, float vel_carrier){
        return jacobian_inverse_ * Eigen::Vector2f(vel_output, vel_carrier);
    }

    Eigen::Vector2f MechanicalDifferential::getOutputVelocity(float vel_1, float vel_2){
        return jacobian_ * Eigen::Vector2f(vel_1, vel_2);
    }

} // namespace ghost_control