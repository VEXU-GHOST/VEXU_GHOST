#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/vector_util.hpp>
#include "ghost_util/unit_conversion_utils.hpp"

using geometry::Line2d;
using ghost_util::angleBetweenVectorsRadians;
namespace ghost_swerve {

SwerveModel::SwerveModel(SwerveConfig config){
	m_config = config;

	validateConfig();
	calculateJacobians();
	calculateMaxBaseTwist();

	// Add each module to module_states map
	for(const auto & [name, _] : m_config.module_positions){
		m_current_module_states[name] = ModuleState();
		m_previous_module_states[name] = ModuleState();
		m_module_commands[name] = ModuleCommand();
	}
}

void SwerveModel::validateConfig(){
	std::unordered_map<std::string, double> double_params{
		{"max_wheel_lin_vel", m_config.max_wheel_lin_vel},
		{"max_lin_vel_slew", m_config.max_lin_vel_slew},
		{"max_ang_vel_slew", m_config.max_ang_vel_slew},
		{"steering_ratio", m_config.steering_ratio},
		{"wheel_ratio", m_config.wheel_ratio},
		{"wheel_radius", m_config.wheel_radius},
		{"steering_kp", m_config.steering_kp},
		{"max_wheel_actuator_vel", m_config.max_wheel_actuator_vel}
	};

	for(const auto& [key, val] : double_params){
		if(val <= 0){
			std::string err_string =
				std::string("[SwerveModel::validateConfig] Error: ") + key + " must be non-zero and positive!";
			throw std::runtime_error(err_string);
		}
	}

	if(m_config.module_positions.size() != 4){
		throw std::runtime_error(
				  "[SwerveModel::validateConfig] Error: module_positions must be of size four (one for each module).");
	}

	m_num_modules = m_config.module_positions.size();
	LIN_VEL_TO_RPM = ghost_util::METERS_TO_INCHES / m_config.wheel_radius * ghost_util::RAD_PER_SEC_TO_RPM;

	m_lin_vel_slew = m_config.max_lin_vel_slew;
	m_ang_slew = m_config.max_ang_vel_slew;

	// Initialize Odometry
	m_odom_loc = Eigen::Vector2d::Zero();
	m_odom_loc.x() = ghost_util::INCHES_TO_METERS * -6.0;
	m_odom_loc.y() = ghost_util::INCHES_TO_METERS * 6.0;
	m_odom_angle = ghost_util::DEG_TO_RAD * 135.0;
}

void SwerveModel::calculateJacobians(){
	switch(m_config.module_type){
		case swerve_type_e::COAXIAL:
		{
			m_module_jacobian << m_config.wheel_ratio, 0.0, 0.0, m_config.steering_ratio;
			m_module_jacobian_inv << 1 / m_config.wheel_ratio, 0.0, 0.0, 1 / m_config.steering_ratio;
		}
		break;

		case swerve_type_e::DIFFERENTIAL:
		{
			m_module_jacobian << m_config.wheel_ratio / 2.0, -m_config.wheel_ratio / 2.0, m_config.steering_ratio / 2.0, m_config.steering_ratio / 2.0;
			m_module_jacobian_inv << 1 / m_config.wheel_ratio, 1 / m_config.steering_ratio, -1 / m_config.wheel_ratio, 1 / m_config.steering_ratio;
		}
		break;
	}

	m_module_jacobian_transpose = m_module_jacobian.transpose();
	m_module_jacobian_inv_transpose = m_module_jacobian_inv.transpose();

	// Task Space Jacobians (Transforms module velocity to and from base velocity)
	m_task_space_jacobian = Eigen::MatrixXd::Zero(3, 2 * m_num_modules);
	m_task_space_jacobian_inverse = Eigen::MatrixXd::Zero(2 * m_num_modules, 3);

	int n = 0;
	for(const auto& [name, position] : m_config.module_positions){
		m_task_space_jacobian_inverse(2 * n, 0) = 1.0;
		m_task_space_jacobian_inverse(2 * n, 2) = -position.y();
		m_task_space_jacobian_inverse(2 * n + 1, 1)  = 1.0;
		m_task_space_jacobian_inverse(2 * n + 1, 2) = position.x();
		n++;
	}

	m_task_space_jacobian = m_task_space_jacobian_inverse.completeOrthogonalDecomposition().pseudoInverse();

	// Ax=b
	// b = [l1*m1_x, l1*m1_y, ..., li*mi_x, li*mi_y]
	// x = [icr_x, icr_y, p1, ..., pi]
	m_least_square_icr_A = Eigen::MatrixXd::Zero(2 * m_num_modules, 2 + m_num_modules);
	m_least_squares_icr_B = Eigen::VectorXd::Zero(2 * m_num_modules);

	n = 0;
	for(const auto& [name, position] : m_config.module_positions){
		m_least_square_icr_A(2 * n, 0) = 1.0;
		m_least_square_icr_A(2 * n + 1, 1) = 1.0;
		m_least_squares_icr_B[2 * n] = position.x();
		m_least_squares_icr_B[2 * n + 1] = position.y();
		n++;
	}
}

void SwerveModel::calculateMaxBaseTwist(){
	// Get Max Base Speeds
	double max_wheel_dist = 0.0;
	for(const auto& [key, val] : m_config.module_positions){
		max_wheel_dist = std::max(max_wheel_dist, (double) val.norm());
	}

	m_max_base_lin_vel = m_config.max_wheel_lin_vel;
	m_max_base_ang_vel = m_max_base_lin_vel / max_wheel_dist;
}

void SwerveModel::setModuleState(const std::string& name, ModuleState state){
	throwOnUnknownSwerveModule(name, "setModuleState");
	state.steering_angle = ghost_util::WrapAngle360(state.steering_angle);
	m_previous_module_states[name] = m_current_module_states[name];
	m_current_module_states[name] = state;
}

// Assumes all module states have been updated prior to update
void SwerveModel::updateSwerveModel(){
	updateBaseTwist();
	calculateLeastSquaresICREstimate();
	calculateOdometry();
}

void SwerveModel::calculateLeastSquaresICREstimate(){
	// Update ICR Jacobian
	int n = 0;
	for(const auto& [name, state] : m_current_module_states){
		m_least_square_icr_A(2 * n, 2 + n) = -cos((state.steering_angle + 90.0) * ghost_util::DEG_TO_RAD);
		m_least_square_icr_A(2 * n + 1, 2 + n) = -sin((state.steering_angle + 90.0) * ghost_util::DEG_TO_RAD);
		n++;
	}
	Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(m_least_square_icr_A);
	auto rank = lu_decomp.rank();
	bool full_rank = (rank == (2 + m_num_modules));

	if(full_rank){
		auto A_decomposition = m_least_square_icr_A.completeOrthogonalDecomposition();
		auto x = A_decomposition.solve(m_least_squares_icr_B);
		m_icr_point = Eigen::Vector2d(x[0], x[1]);

		m_icr_sse = (m_least_square_icr_A * x - m_least_squares_icr_B).norm();
	}

	// Handle pure translation
	m_straight_line_translation = (m_icr_point.norm() > 500.0 || !full_rank);
	if(m_straight_line_translation){
		auto module_state = m_current_module_states.begin()->second;
		double angle = (module_state.steering_angle + 90.0) * ghost_util::DEG_TO_RAD;
		m_icr_point = Eigen::Vector2d(cos(angle), sin(angle));
		m_icr_sse = 0.0;
	}

	double m = 0.0;
	for(const auto& [name, state] : m_current_module_states){
		auto current_steering_axis = Eigen::Vector2d(
			cos((state.steering_angle + 90.0) * ghost_util::DEG_TO_RAD),
			sin((state.steering_angle + 90.0) * ghost_util::DEG_TO_RAD)
			);
		Eigen::Vector2d ideal_steering_axis;

		if(m_straight_line_translation){
			ideal_steering_axis = m_icr_point;
		}
		else{
			ideal_steering_axis = (m_icr_point - m_config.module_positions.at(name)).normalized();
		}

		double angle = angleBetweenVectorsRadians(current_steering_axis, ideal_steering_axis);
		m += angle * angle / (n * M_PI * M_PI);
	}
	double f = 1.0;
	m_icr_quality = 1 - log(f * m + 1) / log(f + 1);
}

void SwerveModel::updateBaseTwist(){
	Eigen::VectorXd module_velocity_vector(2 * m_num_modules);
	int n = 0;
	for(const auto& [name, state] : m_current_module_states){
		module_velocity_vector[2 * n] = state.wheel_velocity / LIN_VEL_TO_RPM * cos(state.steering_angle * ghost_util::DEG_TO_RAD);
		module_velocity_vector[2 * n + 1] = state.wheel_velocity / LIN_VEL_TO_RPM * sin(state.steering_angle * ghost_util::DEG_TO_RAD);
		n++;
	}

	m_base_vel_curr = m_task_space_jacobian * module_velocity_vector;
	m_ls_error_metric = (module_velocity_vector - m_task_space_jacobian_inverse * m_base_vel_curr).norm();

	m_base_vel_curr[0] = (std::fabs(m_base_vel_curr[0]) > 0.01) ? m_base_vel_curr[0] : 0.0;
	m_base_vel_curr[1] = (std::fabs(m_base_vel_curr[1]) > 0.01) ? m_base_vel_curr[1] : 0.0;
	m_base_vel_curr[2] = (std::fabs(m_base_vel_curr[2]) > 0.02) ? m_base_vel_curr[2] : 0.0;
}


void SwerveModel::calculateKinematicSwerveControllerAngleControl(double right_cmd, double forward_cmd, double angle_cmd){
	angle_cmd = ghost_util::WrapAngle2PI(angle_cmd);
	double vel_cmd = ghost_util::SmallestAngleDistRad(angle_cmd, getWorldAngleRad()) * m_config.angle_control_kp;
	calculateKinematicSwerveControllerNormalized(right_cmd / 127.0, forward_cmd / 127.0, -vel_cmd);
}

void SwerveModel::calculateKinematicSwerveControllerMoveToPoseWorld(double des_x, double des_y, double angle_cmd){
	double x_vel = (des_x - getOdometryLocation().x()) * m_config.move_to_pose_kp;
	double y_vel = (des_y - getOdometryLocation().y()) * m_config.move_to_pose_kp;
	calculateKinematicSwerveControllerAngleControl(-y_vel, x_vel, angle_cmd);
}


void SwerveModel::calculateKinematicSwerveControllerJoystick(double right_cmd, double forward_cmd, double clockwise_cmd){
	calculateKinematicSwerveControllerNormalized(right_cmd / 127.0, forward_cmd / 127.0, clockwise_cmd / 127.0);
}

void SwerveModel::calculateKinematicSwerveControllerNormalized(double right_cmd, double forward_cmd, double clockwise_cmd){
	calculateKinematicSwerveControllerVelocity(right_cmd * m_max_base_lin_vel, forward_cmd * m_max_base_lin_vel, clockwise_cmd * m_max_base_ang_vel);
}

void SwerveModel::calculateKinematicSwerveControllerVelocity(double right_cmd, double forward_cmd, double clockwise_cmd){
	// Convert joystick to robot twist command
	Eigen::Vector2d xy_vel_cmd_base_link(forward_cmd, -right_cmd);

	if(m_is_field_oriented){
		// Rotate velocity command to robot frame
		auto rotate_world_to_base = Eigen::Rotation2D<double>(-m_odom_angle).toRotationMatrix();
		xy_vel_cmd_base_link = rotate_world_to_base * xy_vel_cmd_base_link;
	}
	double lin_vel_cmd = std::clamp<double>(xy_vel_cmd_base_link.norm(), -m_max_base_lin_vel, m_max_base_lin_vel);
	double ang_vel_cmd = std::clamp<double>(-clockwise_cmd, -m_max_base_ang_vel, m_max_base_ang_vel);

	// std::cout << std::endl;
	// std::cout << "lin_vel_cmd: " << lin_vel_cmd << std::endl;
	// std::cout << "m_max_base_lin_vel * 0.01: " << m_max_base_lin_vel * 0.01 << std::endl;

	// Zero commands under 1%
	lin_vel_cmd = (fabs(lin_vel_cmd) > m_max_base_lin_vel * 0.01) ? lin_vel_cmd : 0.0;
	ang_vel_cmd = (fabs(ang_vel_cmd) > m_max_base_ang_vel * 0.01) ? ang_vel_cmd : 0.0;

	// std::cout << "lin_vel_cmd: " << lin_vel_cmd << std::endl;

	// For combined linear and angular velocities, we scale down angular velocity (which tends to dominate).
	if((fabs(lin_vel_cmd) > m_max_base_lin_vel * m_config.velocity_scaling_threshold) && (fabs(ang_vel_cmd) > m_max_base_ang_vel * m_config.velocity_scaling_threshold)){
		ang_vel_cmd *= m_config.velocity_scaling_ratio;
	}

	// Calculate Linear Velocity Direction w/ error checking
	Eigen::Vector2d linear_vel_dir(0.0, 0.0);
	if(xy_vel_cmd_base_link.norm() != 0){
		linear_vel_dir = xy_vel_cmd_base_link.normalized();
	}

	// std::cout << "linear_vel_dir: " << linear_vel_dir << std::endl;

	// Update base twist vector
	m_base_vel_cmd = Eigen::Vector3d(
		ghost_util::slewRate(m_base_vel_cmd[0], xy_vel_cmd_base_link.x(), m_lin_vel_slew),
		ghost_util::slewRate(m_base_vel_cmd[1], xy_vel_cmd_base_link.y(), m_lin_vel_slew),
		ghost_util::slewRate(m_base_vel_cmd[2], ang_vel_cmd, m_ang_slew)
		);

	double max_steering_error = 0.0;
	for(const auto& [module_name, module_position] : m_config.module_positions){
		// Calculate linear velocity vector at wheel
		Eigen::Vector2d velocity_vector(0.0, 0.0);
		velocity_vector = ang_vel_cmd * Eigen::Vector2d(-module_position.y(), module_position.x());
		velocity_vector += linear_vel_dir * lin_vel_cmd;

		// Calculate naive steering angle and wheel velocity setpoints
		ModuleCommand module_command;
		module_command.wheel_velocity_vector = velocity_vector;
		module_command.steering_angle_command = ghost_util::WrapAngle360(atan2(velocity_vector.y(), velocity_vector.x()) * ghost_util::RAD_TO_DEG);
		module_command.wheel_velocity_command = velocity_vector.norm() * LIN_VEL_TO_RPM;

		double steering_angle = m_current_module_states.at(module_name).steering_angle;
		double steering_error = ghost_util::SmallestAngleDistDeg(module_command.steering_angle_command, steering_angle);

		if(fabs(steering_error) > 90.0){
			// Flip commands
			module_command.wheel_velocity_command *= -1.0;
			module_command.steering_angle_command = ghost_util::FlipAngle180(module_command.steering_angle_command);

			// Recalculate error
			steering_error = ghost_util::SmallestAngleDistDeg(module_command.steering_angle_command, steering_angle);
		}
		max_steering_error = std::max(max_steering_error, std::fabs(steering_error));

		// Set steering voltage using position control law
		module_command.steering_velocity_command = steering_error * m_config.steering_kp;

		// Update module command
		m_module_commands[module_name] = module_command;
	}

	// TODO(maxxwilson) : FIX using the velocity based Least squares ICR norm
	const double x1 = 50.0;
	// Transient Misalignment Heuristic
	if(max_steering_error > x1){
		// Linear Interpolation past certain angle
		const double x2 = 90.0;
		const double y1 = 1.0;
		const double y2 = 0.0;
		const double slope = (y2 - y1) / (x2 - x1);
		const double intercept = y1 - slope * x1;
		double attentuation_percent = std::max(slope * max_steering_error + intercept, 0.0);
		for(auto& [name, command] : m_module_commands){
			auto module_state = m_current_module_states.at(name);
			m_module_commands[name].wheel_velocity_command *= attentuation_percent;
		}
	}

	for(auto& [name, command] : m_module_commands){
		// Calculate commands in Actuator space
		auto module_vel_cmd = Eigen::Vector2d(command.wheel_velocity_command, command.steering_velocity_command);
		command.actuator_velocity_commands = m_module_jacobian_inv * module_vel_cmd;

		auto module_voltage_cmd = Eigen::Vector2d(command.wheel_voltage_command, command.steering_voltage_command);
		command.actuator_voltage_commands = m_module_jacobian_transpose * module_voltage_cmd;
	}

	if(m_config.module_type == swerve_type_e::DIFFERENTIAL){
		// Get largest velocity magnitude
		double max_velocity = 0.0;
		for(auto& [name, command] : m_module_commands){
			max_velocity = std::max(fabs(command.actuator_velocity_commands[0]), max_velocity);
			max_velocity = std::max(fabs(command.actuator_velocity_commands[1]), max_velocity);
		}

		// Normalize if any motor exceeds maximum possible speed
		if(max_velocity > m_config.max_wheel_actuator_vel){
			for(auto& [name, command] : m_module_commands){
				command.actuator_velocity_commands *= (m_config.max_wheel_actuator_vel / max_velocity);
			}
		}
	}
	else if(m_config.module_type == swerve_type_e::COAXIAL){
		// Get largest velocity magnitude
		double max_velocity = 0.0;
		for(auto& [name, command] : m_module_commands){
			max_velocity = std::max(fabs(command.actuator_velocity_commands[0]), max_velocity);
		}

		// Normalize if any motor exceeds maximum possible speed
		if(max_velocity > m_config.max_wheel_actuator_vel){
			for(auto& [name, command] : m_module_commands){
				command.actuator_velocity_commands[0] *= (m_config.max_wheel_actuator_vel / max_velocity);
			}
		}
	}
	// If we don't receive non-zero user input, zero everything
	if((fabs(lin_vel_cmd) < 1e-5) && (fabs(ang_vel_cmd) < 1e-5)){
		for(auto& [name, command] : m_module_commands){
			command.actuator_velocity_commands = Eigen::Vector2d(0.0, 0.0);
			command.actuator_voltage_commands = Eigen::Vector2d(0.0, 0.0);
		}
	}
}

void SwerveModel::calculateOdometry(){
	auto rotate_base_to_odom = Eigen::Rotation2D<double>(m_odom_angle).toRotationMatrix();
	m_odom_loc += rotate_base_to_odom * Eigen::Vector2d(m_base_vel_curr.x(), m_base_vel_curr.y()) * 0.01;
	m_odom_angle += m_base_vel_curr.z() * 0.01;
	m_odom_angle = ghost_util::WrapAngle2PI(m_odom_angle);
}

void SwerveModel::throwOnUnknownSwerveModule(const std::string& name, const std::string& method_name) const {
	if(m_current_module_states.count(name) == 0){
		throw std::runtime_error (std::string("[SwerveModel::" + method_name +  "] Error:") + name + " is not a known swerve module !");
	}
}

void SwerveModel::setModuleCommand(const std::string& name, ModuleCommand command){
	throwOnUnknownSwerveModule(name, "setModuleCommand");
	command.steering_angle_command = ghost_util::WrapAngle360(command.steering_angle_command);
	m_previous_module_states[name] = m_current_module_states[name];
	m_module_commands[name] = command;
}

const ModuleCommand& SwerveModel::getModuleCommand(const std::string& name){
	throwOnUnknownSwerveModule(name, "getModuleCommand");
	return m_module_commands.at(name);
}

} // namespace ghost_swerve