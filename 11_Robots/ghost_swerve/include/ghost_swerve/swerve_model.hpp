#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
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
	double max_wheel_actuator_vel;

	// Kinematic Controller
	double steering_kp;
	double angle_control_kp;

	// Velocity Scaling
	double velocity_scaling_ratio;
	double velocity_scaling_threshold;

	// Lift - motor angles, not outputs
	double lift_up_angle;
	double lift_speed_rpm;

	// Stick - motor angles, not outputs
	double stick_upright_angle;
	double stick_angle_min;
	double stick_angle_max;
	double stick_turn_offset; 


	// XY Position of each module relative to robot base
	std::map<std::string, Eigen::Vector2d> module_positions;
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

	Eigen::Vector2d wheel_velocity_vector = Eigen::Vector2d(0.0, 0.0);
	Eigen::Vector2d actuator_velocity_commands = Eigen::Vector2d(0.0, 0.0);
	Eigen::Vector2d actuator_voltage_commands = Eigen::Vector2d(0.0, 0.0);

	ModuleCommand() = default;

	bool operator==(const ModuleCommand& rhs) const {
		return (std::fabs(wheel_velocity_command - rhs.wheel_velocity_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(wheel_voltage_command - rhs.wheel_voltage_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_angle_command - rhs.steering_angle_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_velocity_command - rhs.steering_velocity_command) < std::numeric_limits<double>::epsilon()) &&
		       (std::fabs(steering_voltage_command - rhs.steering_voltage_command) < std::numeric_limits<double>::epsilon()) &&
		       (actuator_velocity_commands.isApprox(rhs.actuator_velocity_commands)) &&
		       (actuator_voltage_commands.isApprox(rhs.actuator_voltage_commands));
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
	const ModuleState& getCurrentModuleState(const std::string& name){
		throwOnUnknownSwerveModule(name, "getCurrentModuleState");
		return m_current_module_states.at(name);
	}

	/**
	 * @brief Return the previous state of a given Swerve Module.
	 *
	 * @param name
	 * @return const ModuleState&
	 */
	const ModuleState& getPreviousModuleState(const std::string& name){
		throwOnUnknownSwerveModule(name, "getPreviousModuleState");
		return m_previous_module_states.at(name);
	}

	/**
	 * @brief Updates a swerve module command by module name.
	 *
	 * @param name
	 * @param command
	 */
	void setModuleCommand(const std::string& name, ModuleCommand command);

	/**
	 * @brief Gets a swerve module command by name.
	 *
	 * @param name
	 * @param state
	 */
	const ModuleCommand& getModuleCommand(const std::string& name);

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
	 * @brief Updates module wheel and steering setpoints given a base twist.
	 *
	 * @param right
	 * @param forward
	 * @param clockwise
	 */
	void calculateKinematicSwerveControllerNormalized(double right_cmd, double forward_cmd, double clockwise_cmd);
	void calculateKinematicSwerveControllerJoystick(double right_cmd, double forward_cmd, double clockwise_cmd);
	void calculateKinematicSwerveControllerVelocity(double right_cmd, double forward_cmd, double clockwise_cmd);

	void calculateKinematicSwerveControllerAngleControl(double right_cmd, double forward_cmd, double angle_cmd);


	const Eigen::Vector3d& getBaseVelocityCommand(){
		return m_base_vel_cmd;
	}

	const Eigen::Vector3d& getBaseVelocityCurrent(){
		return m_base_vel_curr;
	}

	double getICRSSE() const {
		return m_icr_sse;
	}

	double getICRQuality() const {
		return m_icr_quality;
	}

	const Eigen::Vector2d& getOdometryLocation(){
		return m_odom_loc;
	}

	double getOdometryAngle() const {
		return m_odom_angle;
	}

	const Eigen::Vector2d& getWorldLocation(){
		// TODO(maxxwilson) UPDATE THIS from sub
		return m_odom_loc;
	}

	double getWorldAngleDeg() const {
		// TODO(maxxwilson) UPDATE THIS from sub
		return m_odom_angle * ghost_util::RAD_TO_DEG;
	}

	double getWorldAngleRad() const {
		// TODO(maxxwilson) UPDATE THIS from sub
		return m_odom_angle;
	}

	void setFieldOrientedControl(bool field_oriented_control){
		m_is_field_oriented = field_oriented_control;
	}

	bool isFieldOrientedControl() const {
		return m_is_field_oriented;
	}

protected:
	// Initialization
	void validateConfig();
	void calculateJacobians();
	void calculateMaxBaseTwist();

	// Error Catching
	void throwOnUnknownSwerveModule(const std::string& name, const std::string& method_name) const;

	// Model Updates
	void calculateLeastSquaresICREstimate();
	void calculateOdometry();
	void updateBaseTwist();
	void updateICREstimate();

	// Configuration
	SwerveConfig m_config;
	double m_max_base_lin_vel;
	double m_max_base_ang_vel;
	int m_num_modules;
	double LIN_VEL_TO_RPM;

	// Jacobians
	Eigen::Matrix2d m_module_jacobian;                      // Maps actuator velocities to joint velocities
	Eigen::Matrix2d m_module_jacobian_inv;                  // Maps joint velocities to actuator velocities
	Eigen::Matrix2d m_module_jacobian_transpose;            // Maps actuator efforts to joint efforts
	Eigen::Matrix2d m_module_jacobian_inv_transpose;        // Maps joint efforts to actuator efforts

	Eigen::MatrixXd m_task_space_jacobian;                  // Maps Joint Velocities to Base Velocities
	Eigen::MatrixXd m_task_space_jacobian_inverse;          // Maps Base Velocities to Joint Velocities

	Eigen::MatrixXd m_least_square_icr_A;
	Eigen::VectorXd m_least_squares_icr_B;

	// Odometry
	Eigen::Vector2d m_odom_loc;
	double m_odom_angle;

	// Current centroidal states
	double m_curr_angle;
	Eigen::Vector3d m_base_vel_curr;

	// Command Setpoints
	Eigen::Vector3d m_base_vel_cmd;

	// Module States
	std::map <std::string, ModuleState> m_previous_module_states;
	std::map <std::string, ModuleState> m_current_module_states;
	std::map <std::string, ModuleCommand> m_module_commands;

	// Control Style
	bool m_is_field_oriented = false;

	// ICR States
	Eigen::Vector3d m_h_space_projection;
	Eigen::Vector2d m_icr_point;
	bool m_straight_line_translation;
	double m_icr_quality = 0;
	double m_icr_sse = 0;
};

} // namespace ghost_swerve