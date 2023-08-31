/*
 * Filename: mechanical_differential
 * Created Date: Thursday May 11th 2023
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Thursday May 11th 2023 12:53:41 am
 * Modified By: Maxx Wilson
 */

#ifndef GHOST_CONTROL__MECHANICAL_DIFFERENTIAL_HPP
#define GHOST_CONTROL__MECHANICAL_DIFFERENTIAL_HPP

#include "eigen3/Eigen/Geometry"

namespace ghost_control {

class MechanicalDifferential {
public:
	MechanicalDifferential(){
	}
	MechanicalDifferential(float carrier_ratio, float output_ratio);

	Eigen::Matrix2f getJacobian(){
		return jacobian_;
	}

	Eigen::Matrix2f getJacobianInverse(){
		return jacobian_inverse_;
	}

	Eigen::Matrix2f getJacobianTranspose(){
		return jacobian_transpose_;
	}

	Eigen::Matrix2f getJacobianInverseTranspose(){
		return jacobian_inverse_transpose_;
	}

	Eigen::Vector2f getInputTorque(float tau_output, float tau_carrier);
	Eigen::Vector2f getOutputTorque(float tau_1, float tau_2);
	Eigen::Vector2f getInputVelocity(float vel_output, float vel_carrier);
	Eigen::Vector2f getOutputVelocity(float vel_1, float vel_2);

private:
	float carrier_ratio_;
	float output_ratio_;

	Eigen::Matrix2f jacobian_;
	Eigen::Matrix2f jacobian_inverse_;
	Eigen::Matrix2f jacobian_transpose_;
	Eigen::Matrix2f jacobian_inverse_transpose_;
};

} // namespace ghost_control
#endif