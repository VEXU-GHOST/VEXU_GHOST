#pragma once

#include <string>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"
#include "math/line2d.h"


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

	/**
	 * @brief Updates a swerve module state by module name.
	 *
	 * @param name
	 * @param state
	 */
	void setModuleState(const std::string& name, ModuleState state);

	/**
	 * @brief Calculates various attributes of the swerve model based on the current Module States. Call after updating
	 * all modules with new sensor data.
	 */
	void updateSwerveModel();

	/**
	 * @brief Return current state of a given Swerve Module.
	 *
	 * @param name
	 * @return const ModuleState&
	 */
	const ModuleState& getCurrentModuleState(const std::string& name);

	/**
	 * @brief Return the previous state of a given Swerve Module.
	 *
	 * @param name
	 * @return const ModuleState&
	 */
	const ModuleState& getPreviousModuleState(const std::string& name);

	/// Module Jacobians ///

	/**
	 * @brief Return the 2x2 Jacobian Matrix which maps from actuator to joint velocities.
	 * This will convert motor states to wheel/steering states.
	 *
	 * @return const Eigen::Matrix2d&
	 */
	const Eigen::Matrix2d& getModuleJacobian() const {
		return m_module_jacobian;
	}

	/**
	 * @brief Return the 2x2 Jacobian Matrix which maps from joint to actuator velocities.
	 * This will convert wheel/steering states to motor states.
	 *
	 * @return const Eigen::Matrix2d&
	 */
	const Eigen::Matrix2d& getModuleJacobianInverse() const {
		return m_module_jacobian_inv;
	}

	/**
	 * @brief Return the 2x2 Jacobian Matrix transpose which maps from joint to actuator torques.
	 * This will convert wheel/steering torques to motor torques.
	 *
	 * @return const Eigen::Matrix2d&
	 */
	const Eigen::Matrix2d& getModuleJacobianTranspose() const {
		return m_module_jacobian_transpose;
	}

	/**
	 * @brief Return the 2x2 Jacobian Matrix transpose which maps from actuator to joint torques.
	 * This will convert motor torques to wheel/steering torques.
	 *
	 * @return const Eigen::Matrix2d&
	 */
	const Eigen::Matrix2d& getModuleJacobianInverseTranspose() const {
		return m_module_jacobian_inv_transpose;
	}

	/**
	 * @brief Get the max linear velocity of the robot base at nominal motor speed.
	 *
	 * @return double
	 */
	double getMaxBaseLinearVelocity() const {
		return m_max_base_lin_vel;
	}

	/**
	 * @brief Get the max angular velocity of the robot base at nominal motor speed.
	 *
	 * @return double
	 */
	double getMaxBaseAngularVelocity() const {
		return m_max_base_ang_vel;
	}

	/**
	 * @brief Gets the current estimate for the Instant Center of Rotation in the 2D plane.
	 * If the ICR is at infinity (i.e. robot is driving perfectly straight), we return true and return a unit vector
	 * in the direction of the ICR.
	 *
	 * @param icr_point
	 * @return true if ICR is at infinity
	 * @return false otherwise
	 */
	bool getICR(Eigen::Vector2d& icr_point){
		icr_point = m_icr_point;
		return m_icr_is_inf;
	}

	/**
	 * @brief Returns a projection of the 2D Instant Center of Rotation onto a unit sphere positioned beneath the robot.
	 * This is referred to in literature as the "H-Space" projection, and allows for ICR motion planning without
	 * discontinuities when driving straight (causing 2D ICR to go to infinity).
	 *
	 * @return const Eigen::Vector3d&
	 */
	const Eigen::Vector3d& getSphericalICRProjection(){
		return m_h_space_projection;
	}


protected:
	// Initialization
	void validateConfig();
	void calculateJacobians();
	void calculateMaxBaseTwist();

	void throwOnUnknownSwerveModule(const std::string& name, const std::string& method_name) const;
	void calculateOdometry();

	void calculateHSpaceICR(std::vector<geometry::Line2d> wheel_axes, Eigen::Vector2d& icr);


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
	std::unordered_map <std::string, ModuleState> m_previous_module_states;
	std::unordered_map <std::string, ModuleState> m_current_module_states;

	// Control Style
	bool m_is_angle_control = false;
	bool m_is_field_oriented = false;

	// ICR States
	Eigen::Vector3d m_h_space_projection;
	Eigen::Vector2d m_icr_point;
	bool m_icr_is_inf;
};

} // namespace ghost_swerve