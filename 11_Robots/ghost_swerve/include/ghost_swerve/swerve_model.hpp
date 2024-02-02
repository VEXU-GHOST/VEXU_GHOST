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
	double steering_ratio;
	double wheel_ratio;

	// XY Position of each module relative to robot base
	std::unordered_map<std::string, Eigen::Vector2d> module_positions;
};

struct ModuleState {
	double wheel_position;
	double wheel_velocity;
	double steering_position;
	double steering_velocity;

	bool operator==(const ModuleState& rhs) const {
		return (std::fabs(wheel_position - rhs.wheel_position) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(wheel_velocity - rhs.wheel_velocity) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_position - rhs.steering_position) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_velocity - rhs.steering_velocity) < std::numeric_limits<double>::epsilon());
	}
};

class SwerveModel {
public:
	SwerveModel(SwerveConfig config);
	void updateRobotStates(const std::unordered_map<std::string, Eigen::Vector2d>& joint_positions,
	                       const std::unordered_map<std::string, Eigen::Vector2d>& joint_velocities);

	void updateRobotStates(const std::unordered_map<std::string, Eigen::Vector2d>& joint_positions,
	                       const std::unordered_map<std::string, Eigen::Vector2d>& joint_velocities,
	                       const std::unordered_map<std::string, double>& steering_positions,
	                       const std::unordered_map<std::string, double>& steering_velocities);

	void updateSwerveCommandsFromTwist(Eigen::Vector3d twist_cmd);
	const ModuleState& getModuleState(const std::string& name);

	const Eigen::Matrix2d& getModuleJacobian() const {
		return m_module_jacobian;
	}

	const Eigen::Matrix2d& getModuleJacobianInverse() const {
		return m_module_jacobian_inv;
	}

	const Eigen::Matrix2d& getModuleJacobianTranspose() const {
		return m_module_jacobian_transpose;
	}

	const Eigen::Matrix2d& getModuleJacobianInverseTranspose() const {
		return m_module_jacobian_inv_transpose;
	}

	double getMaxBaseLinearVelocity() const {
		return m_max_base_lin_vel;
	}

	double getMaxBaseAngularVelocity() const {
		return m_max_base_ang_vel;
	}

protected:
	void validateConfig();
	void calculateJacobians();
	void calculateMaxBaseTwist();
	void throwOnUnknownSwerveModule(const std::string& name, const std::string& method_name) const;
	void calculateHSpaceICR();
	void calculateOdometry();

	// Configuration
	SwerveConfig m_config;
	double m_max_base_lin_vel;
	double m_max_base_ang_vel;

	// Jacobians
	Eigen::Matrix2d m_module_jacobian;
	Eigen::Matrix2d m_module_jacobian_inv;
	Eigen::Matrix2d m_module_jacobian_transpose;
	Eigen::Matrix2d m_module_jacobian_inv_transpose;

	// Odometry
	Eigen::Vector3d m_odom_pose;

	// Current centroidal states
	double m_curr_angle;
	Eigen::Vector3d m_base_vel_curr;

	// Command Setpoints
	Eigen::Vector3d m_base_vel_cmd;

	// Module States
	std::unordered_map <std::string, ModuleState> m_last_module_states;
	std::unordered_map <std::string, ModuleState> m_current_module_states;

	// Control Style
	bool m_is_angle_control = false;
	bool m_is_field_oriented = false;
};

} // namespace ghost_swerve