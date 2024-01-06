#pragma once

#include <string>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"

namespace ghost_swerve {

enum swerve_type_e {
	COAXIAL,
	DIFFERENTIAL
};

struct SwerveConfig {
	// Maximum linear speed of robot base (or linear velocity of wheel surface).
	double max_wheel_lin_vel;

	// Module Config
	swerve_type_e module_type;
	float steering_ratio;
	float wheel_ratio;

	// XY Position of each module relative to robot base
	std::unordered_map<std::string, Eigen::Vector2f> module_positions;
};

struct ModuleState {
	Eigen::Vector2f pos_joints;
	Eigen::Vector2f vel_joints;
	Eigen::Vector2f accel_joints;
};

class SwerveModel {
public:
	SwerveModel(SwerveConfig config);
	void calculateHSpaceICR();
	void calculateOdometry();
	void updateSwerveCommandsFromTwist(Eigen::Vector3f twist_cmd);

	const Eigen::Matrix2f& getModuleJacobian() const {
		return module_jacobian_;
	}

	const Eigen::Matrix2f& getModuleJacobianInverse() const {
		return module_jacobian_inv_;
	}

	const Eigen::Matrix2f& getModuleJacobianTranspose() const {
		return module_jacobian_transpose_;
	}

	const Eigen::Matrix2f& getModuleJacobianInverseTranspose() const {
		return module_jacobian_inv_transpose_;
	}

	double getMaxBaseLinearVelocity() const {
		return max_base_lin_vel_;
	}

	double getMaxBaseAngularVelocity() const {
		return max_base_ang_vel_;
	}

protected:
	void validateConfig();
	void calculateJacobians();
	void calculateMaxBaseTwist();

	// Configuration
	SwerveConfig config_;
	double max_base_lin_vel_;
	double max_base_ang_vel_;

	// Jacobians
	Eigen::Matrix2f module_jacobian_;
	Eigen::Matrix2f module_jacobian_inv_;
	Eigen::Matrix2f module_jacobian_transpose_;
	Eigen::Matrix2f module_jacobian_inv_transpose_;

	// Centroidal states in World Frame
	Eigen::Vector3f base_pos_curr_;
	Eigen::Vector3f base_vel_curr_;
	Eigen::Vector3f base_accel_curr_;

	// Command Setpoints
	float base_angle_cmd_;
	Eigen::Vector3f base_vel_cmd_;

	// Module States
	std::unordered_map <std::string, ModuleState> module_states;

	// Control Style
	bool is_angle_control = false;
	bool is_field_oriented = false;
};

} // namespace ghost_swerve