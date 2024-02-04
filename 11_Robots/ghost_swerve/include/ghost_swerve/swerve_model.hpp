#pragma once

#include <string>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"
#include <ghost_util/angle_util.hpp>
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
	double wheel_radius;

	// XY Position of each module relative to robot base
	std::unordered_map<std::string, Eigen::Vector2d> module_positions;
};

struct ModuleState {
	double wheel_position;
	double wheel_velocity;
	double wheel_acceleration;
	double steering_angle;
	double steering_velocity;
	double steering_acceleration;

	ModuleState() = default;

	ModuleState(double wheel_pos, double steering_ang, double wheel_vel, double steering_vel, double wheel_accel = 0.0, double steering_accel = 0.0){
		wheel_position = wheel_pos;
		steering_angle = ghost_util::WrapAngle360(steering_ang);
		wheel_velocity = wheel_vel;
		steering_velocity = steering_vel;
		wheel_acceleration = wheel_accel;
		steering_acceleration = steering_accel;
	}

	bool operator==(const ModuleState& rhs) const {
		return (std::fabs(wheel_position - rhs.wheel_position) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(wheel_velocity - rhs.wheel_velocity) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(wheel_acceleration - rhs.wheel_acceleration) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_angle - rhs.steering_angle) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_velocity - rhs.steering_velocity) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_acceleration - rhs.steering_acceleration) < std::numeric_limits<double>::epsilon());
	}
};

struct ModuleCommand {
	double wheel_velocity_command;
	double wheel_voltage_command;
	double steering_angle_command;
	double steering_velocity_command;
	double steering_voltage_command;

	ModuleCommand() = default;

	ModuleCommand(double wheel_vel_cmd, double wheel_vlt_cmd, double steering_pos_cmd, double steering_vel_cmd, double steering_vlt_cmd){
		wheel_velocity_command = wheel_vel_cmd;
		wheel_voltage_command = wheel_vlt_cmd;
		steering_angle_command = ghost_util::WrapAngle360(steering_pos_cmd);
		steering_velocity_command = steering_vel_cmd;
		steering_voltage_command = steering_vlt_cmd;
	}

	bool operator==(const ModuleCommand& rhs) const {
		return (std::fabs(wheel_velocity_command - rhs.wheel_velocity_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(wheel_voltage_command - rhs.wheel_voltage_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_angle_command - rhs.steering_angle_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_velocity_command - rhs.steering_velocity_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_voltage_command - rhs.steering_voltage_command) < std::numeric_limits<double>::epsilon());
	}
};

class SwerveModel {
public:
	SwerveModel(SwerveConfig config);

	/**
	 * @brief Get the Swerve Model Configration
	 *
	 * @return const SwerveConfig&
	 */
	const SwerveConfig& getConfig(){
		return m_config;
	}

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
		return m_straight_line_translation;
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

	/**
	 * @brief Updates module wheel and steering setpoints given a normalized base twist (each dimension is -1.0 -> 1.0).
	 * This is internally scaled by the maximum linear/angular velocities of the robot.
	 *
	 * @param right_vel
	 * @param forward_vel
	 * @param clockwise_vel
	 */
	void calculateKinematicSwerveController(double right_vel, double forward_vel, double clockwise_vel);

	std::vector<geometry::Line2d> calculateWheelAxisVectors() const;
	static std::vector<Eigen::Vector3d> calculateSphericalProjectionAxisIntersections(std::vector<geometry::Line2d> axes);
	static Eigen::Vector3d averageVectorAntipoles(std::vector<Eigen::Vector3d> vectors);
	void filterCollinearVectors(std::vector<Eigen::Vector3d>& vectors, int num_modules);
	void calculateHSpaceICR();

protected:
	// Initialization
	void validateConfig();
	void calculateJacobians();
	void calculateMaxBaseTwist();

	void throwOnUnknownSwerveModule(const std::string& name, const std::string& method_name) const;
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
	std::unordered_map <std::string, ModuleState> m_previous_module_states;
	std::unordered_map <std::string, ModuleState> m_current_module_states;
	std::unordered_map <std::string, ModuleCommand> m_module_commands;

	// Control Style
	bool m_is_angle_control = false;
	bool m_is_field_oriented = false;

	// ICR States
	Eigen::Vector3d m_h_space_projection;
	Eigen::Vector2d m_icr_point;
	bool m_straight_line_translation;
};

} // namespace ghost_swerve