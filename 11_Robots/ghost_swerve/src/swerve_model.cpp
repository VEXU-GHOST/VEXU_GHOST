#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/vector_util.hpp>
#include "ghost_util/unit_conversion_utils.hpp"

using geometry::Line2d;

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

const ModuleState& SwerveModel::getCurrentModuleState(const std::string& name){
	throwOnUnknownSwerveModule(name, "getCurrentModuleState");
	return m_current_module_states.at(name);
}

const ModuleState& SwerveModel::getPreviousModuleState(const std::string& name){
	throwOnUnknownSwerveModule(name, "getPreviousModuleState");
	return m_previous_module_states.at(name);
}

void SwerveModel::setModuleState(const std::string& name, ModuleState state){
	throwOnUnknownSwerveModule(name, "setModuleState");
	state.steering_angle = ghost_util::WrapAngle360(state.steering_angle);
	m_previous_module_states[name] = m_current_module_states[name];
	m_current_module_states[name] = state;
}

void SwerveModel::updateSwerveModel(){
	calculateHSpaceICR();
	calculateOdometry();
}

void SwerveModel::calculateKinematicSwerveController(double right_vel, double forward_vel, double clockwise_vel){
	// Convert joystick to robot twist command
	Eigen::Vector2d xy_vel_cmd_base_link(forward_vel, -right_vel);
	double lin_vel_cmd = std::clamp<double>(xy_vel_cmd_base_link.norm(), -1.0, 1.0) * m_max_base_lin_vel;
	double ang_vel_cmd = std::clamp<double>(clockwise_vel, -1.0, 1.0) * m_max_base_ang_vel;

	// Zero commands under 1%
	lin_vel_cmd = (fabs(lin_vel_cmd) > m_max_base_lin_vel * 0.01) ? lin_vel_cmd : 0.0;
	ang_vel_cmd = (fabs(ang_vel_cmd) > m_max_base_ang_vel * 0.01) ? ang_vel_cmd : 0.0;

	// Calculate Linear Velocity Direction w/ error checking
	Eigen::Vector2d linear_vel_dir(0.0, 0.0);
	if(xy_vel_cmd_base_link.norm() != 0){
		linear_vel_dir = xy_vel_cmd_base_link.normalized();
	}

	// Update base twist vector
	m_base_vel_cmd = Eigen::Vector3d(xy_vel_cmd_base_link.x(), xy_vel_cmd_base_link.y(), ang_vel_cmd);

	const double LIN_VEL_TO_RPM = ghost_util::METERS_TO_INCHES / m_config.wheel_radius * ghost_util::RAD_PER_SEC_TO_RPM;
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

	const double x1 = 20.0;
	// Transient Misalignment Heuristic
	if(max_steering_error > x1){
		// Linear Interpolation past certain angle
		const double x2 = 45.0;
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
	for(auto& [name, command] : m_module_commands){
	}
	// If we don't receive non-zero user input, zero everything
	if((fabs(lin_vel_cmd) < 1e-5) && (fabs(ang_vel_cmd) < 1e-5)){
		for(auto& [name, command] : m_module_commands){
			command.actuator_velocity_commands = Eigen::Vector2d(0.0, 0.0);
			command.actuator_voltage_commands = Eigen::Vector2d(0.0, 0.0);
		}
	}
}

std::vector<Line2d> SwerveModel::calculateWheelAxisVectors() const {
	std::vector<Line2d> axis_vectors;

	// Convert each wheel module to Line2d
	for(const auto & [name, start_point] : m_config.module_positions){
		// Get angle of module in radians
		double angle_rad = (m_current_module_states.at(name).steering_angle) * M_PI / 180.0;

		// Calculate end point of unit vector, rotating vector by 90 to align with driveshaft
		auto end_point = start_point + Eigen::Vector2d(-sin(angle_rad), cos(angle_rad));
		axis_vectors.push_back(Line2d(start_point, end_point));
	}

	return axis_vectors;
}

std::vector<Eigen::Vector3d> SwerveModel::calculateSphericalProjectionAxisIntersections(std::vector<Line2d> axes){
	if(axes.size() < 2){
		throw std::runtime_error("[SwerveModel::calculateSphericalProjectionAxisIntersections] Error: Requires atleast two axes to find intersections!");
	}

	std::vector<Eigen::Vector3d> intersection_points;
	for(int i = 0; i < axes.size(); i++){
		auto l1 = axes[i];
		for(int j = i + 1; j < axes.size(); j++){
			auto l2 = axes[j];
			if(fabs(geometry::Cross(l1.Dir(), l2.Dir())) < 1e-9){
				intersection_points.push_back(Eigen::Vector3d(l1.Dir().x(), l1.Dir().y(), 0.0));
			}
			else{
				Eigen::Hyperplane<double, 2> hp1 = Eigen::Hyperplane<double, 2>::Through(l1.p0, l1.p1);
				Eigen::Hyperplane<double, 2> hp2 = Eigen::Hyperplane<double, 2>::Through(l2.p0, l2.p1);
				auto intersection = hp1.intersection(hp2);
				auto intersection_3d = Eigen::Vector3d(intersection.x(), intersection.y(), 1);
				intersection_points.push_back(intersection_3d / intersection_3d.norm());
			}
		}
	}

	return intersection_points;
}

double SwerveModel::averageVectorAntipoles(std::vector<Eigen::Vector3d> vectors, Eigen::Vector3d& avg_point, int num_rejected_points){
	if(vectors.size() == 0){
		throw std::runtime_error("[SwerveModel::averageVectorAntipoles] Error: vector list is empty!");
	}

	// Find fusion candidates
	auto first_vector = vectors[0].normalized();
	std::vector<Eigen::Vector3d> candidates{first_vector};
	for(int i = 1; i < vectors.size(); i++){
		auto p = vectors[i].normalized();
		auto d1 = ghost_util::angleBetweenVectorsRadians<Eigen::Vector3d>(first_vector, p);
		auto d2 = ghost_util::angleBetweenVectorsRadians<Eigen::Vector3d>(first_vector,-p);
		if(d1 < d2){
			candidates.push_back(p);
		}
		else{
			candidates.push_back(-p);
		}
	}

	// Calculate average
	Eigen::Vector3d avg(0, 0, 0);
	for(const auto &p : candidates){
		avg += p;
	}
	avg_point = (avg / candidates.size()).normalized();

	// // Reject points with largest distance
	// for(int i = 0; i < num_rejected_points; i++){
	// 	double max_err = 0.0;
	// 	int max_err_index = -1;
	// 	for(int j = 0; j < candidates.size() - i; j++){
	// 		auto err = (candidates[j] - avg_point).squaredNorm();
	// 		if(err > max_err){
	// 			max_err = err;
	// 			max_err_index = j;
	// 		}
	// 	}
	// 	candidates.erase(candidates.begin() + max_err_index);
	// }

	// // Re-average
	// std::cout << "Reaverage" << std::endl;
	// avg = Eigen::Vector3d(0, 0, 0);
	// for(const auto &p : candidates){
	// 	std::cout << p << std::endl;
	// 	avg += p;
	// }
	// avg_point = (avg / candidates.size()).normalized();

	// Calculate sum squared error
	double sse = 0.0;
	// std::cout << "avg point" << std::endl;
	// std::cout << avg_point << std::endl;
	for(const auto &p : candidates){
		// std::cout << "p" << std::endl;
		// std::cout << p << std::endl;
		sse += (avg_point - p).squaredNorm();
	}

	return sse;
}

void SwerveModel::calculateHSpaceICR(){
	std::vector<Line2d>  axis_vectors = calculateWheelAxisVectors();
	std::vector<Eigen::Vector3d> h_space_unit_vectors = calculateSphericalProjectionAxisIntersections(axis_vectors);
	filterCollinearVectors(h_space_unit_vectors, axis_vectors.size());
	m_icr_sse = averageVectorAntipoles(h_space_unit_vectors, m_h_space_projection, axis_vectors.size() / 2);

	// Update 2D ICR
	if(fabs(m_h_space_projection[2]) < 1e-9){
		// Handle Parallel Case (Straight line translation)
		m_icr_point = Eigen::Vector2d(m_h_space_projection[0], m_h_space_projection[1]).normalized();
		m_straight_line_translation = true;
	}
	else{
		m_icr_point = Eigen::Vector2d(m_h_space_projection[0] / m_h_space_projection[2], m_h_space_projection[1] / m_h_space_projection[2]);
		m_straight_line_translation = false;
	}
}

void SwerveModel::filterCollinearVectors(std::vector<Eigen::Vector3d>& vectors, int num_modules){
	// If axes are collinear but the robot isn't moving straight, it throws off the ICR average (and thus all
	// related calculations). Store the indices of collinear vectors (in reverse, if we need to erase them later).
	std::vector<int> collinear_vector_indices;
	std::vector<int> non_collinear_vector_indices;
	for(int i = vectors.size() - 1; i >= 0; i--){
		if(vectors[i][2] < 1e-3){
			collinear_vector_indices.push_back(i);
		}
		else{
			non_collinear_vector_indices.push_back(i);
		}
	}

	// If we aren't driving straight, the maximum number of collinear vectors is num_modules / 2,
	// where the ICR is in center of the robot, and each pair of opposite modules intersects.
	if((collinear_vector_indices.size() != 0) && (collinear_vector_indices.size() <= num_modules / 2)){
		std::cout << "collinear filtered: " << collinear_vector_indices.size() << std::endl;
		for(const auto & index : collinear_vector_indices){
			vectors.erase(vectors.begin() + index);
		}
	}

	// If we are driving straight (within threshold), then there can be up to two modules which roughly line up but don't evaluate
	// to collinear due to small errors. I.e. ICR points to the left, both right module axes point at their respective left module axes.
	// There are two intersection points inside the robot chassis. From small misalignments.
	if((non_collinear_vector_indices.size() != 0) && (non_collinear_vector_indices.size() <= num_modules / 2)){
		std::cout << "non collinear filtered: " << non_collinear_vector_indices.size() << std::endl;
		for(const auto & index : non_collinear_vector_indices){
			vectors.erase(vectors.begin() + index);
		}
	}
}

template <typename T>
std::vector<std::vector<T> > getUniqueAntipoleSets(std::vector<T> vectors){
	std::vector<std::vector<T> > unique_sets;
	for(auto & v : vectors){
		unique_sets.push_back(std::vector<T>{v});
	}
	for(int i = 1; i < vectors.size(); i++){
		unique_sets[0].push_back(vectors[i]);
	}

	for(int i = 1; i < vectors.size(); i++){
		unique_sets[i].push_back(vectors[i]);
		for(int j = i; j < vectors.size(); j++){
		}
	}
}


void SwerveModel::calculateOdometry(){
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