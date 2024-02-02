#include <ghost_swerve/swerve_model.hpp>


namespace ghost_swerve {

SwerveModel::SwerveModel(SwerveConfig config){
	m_config = config;

	validateConfig();
	calculateJacobians();
	calculateMaxBaseTwist();

	// Add each module to module_states map
	for(const auto & [name, _] : m_config.module_positions){
		m_current_module_states[name] = ModuleState();
	}
}

void SwerveModel::validateConfig(){
	std::unordered_map<std::string, double> double_params{
		{"max_wheel_lin_vel", m_config.max_wheel_lin_vel},
		{"steering_ratio", m_config.steering_ratio},
		{"wheel_ratio", m_config.wheel_ratio},
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

const ModuleState& SwerveModel::getModuleState(const std::string& name){
	throwOnUnknownSwerveModule(name, "getModuleState");
	return m_current_module_states.at(name);
}

void SwerveModel::updateRobotStates(const std::unordered_map<std::string, Eigen::Vector2d>& joint_positions,
                                    const std::unordered_map<std::string, Eigen::Vector2d>& joint_velocities){
	if(joint_positions.size() != joint_velocities.size()){
		throw std::runtime_error("[SwerveModel::updateRobotStates] Error: Position and Velocity maps must be the same size!");
	}

	for(const auto& [name, _] : joint_positions){
		throwOnUnknownSwerveModule(name, "updateRobotStates");
		if(joint_velocities.count(name) == 0){
			throw std::runtime_error(std::string("[SwerveModel::updateRobotStates] Error: ") + name + " is not in the joint_velocities map!");
		}

		Eigen::Vector2d module_pos = m_module_jacobian * joint_positions.at(name);
		Eigen::Vector2d module_vel = m_module_jacobian * joint_velocities.at(name);
		m_current_module_states[name].wheel_position = module_pos[0];
		m_current_module_states[name].wheel_velocity = module_vel[0];
		m_current_module_states[name].steering_position = module_pos[1];
		m_current_module_states[name].steering_velocity = module_vel[1];
	}
}

void SwerveModel::updateRobotStates(const std::unordered_map<std::string, Eigen::Vector2d>& joint_positions,
                                    const std::unordered_map<std::string, Eigen::Vector2d>& joint_velocities,
                                    const std::unordered_map<std::string, double>& steering_positions,
                                    const std::unordered_map<std::string, double>& steering_velocities){
	updateRobotStates(joint_positions, joint_velocities);

	if(steering_positions.size() != steering_velocities.size()){
		throw std::runtime_error("[SwerveModel::updateRobotStates] Error: Steering Position and Steering Velocity maps must be the same size!");
	}

	for(const auto& [name, _] : steering_positions){
		throwOnUnknownSwerveModule(name, "updateRobotStates");
		if(steering_velocities.count(name) == 0){
			throw std::runtime_error(std::string("[SwerveModel::updateRobotStates] Error: ") + name + " is not in the steering_velocities map!");
		}

		m_current_module_states[name].steering_position = steering_positions.at(name);
		m_current_module_states[name].steering_velocity = steering_velocities.at(name);
	}
}

void SwerveModel::calculateOdometry(){
}

void SwerveModel::updateSwerveCommandsFromTwist(Eigen::Vector3d twist_cmd){
}

void SwerveModel::throwOnUnknownSwerveModule(const std::string& name, const std::string& method_name) const {
	if(m_current_module_states.count(name) == 0){
		throw std::runtime_error (std::string("[SwerveModel::" + method_name +  "] Error:") + name + " is not a known swerve module !");
	}
}

} // namespace ghost_swerve