#include <ghost_swerve/swerve_model.hpp>


namespace ghost_swerve {

SwerveModel::SwerveModel(SwerveConfig config){
	config_ = config;

	validateConfig();
	calculateJacobians();
	calculateMaxBaseTwist();
}

void SwerveModel::validateConfig(){
	std::unordered_map<std::string, float> float_params{
		{"max_wheel_lin_vel", config_.max_wheel_lin_vel},
		{"steering_ratio", config_.steering_ratio},
		{"wheel_ratio", config_.wheel_ratio},
	};

	for(const auto& [key, val] : float_params){
		if(val <= 0){
			std::string err_string =
				std::string("[SwerveModel::validateConfig] Error: ") + key + " must be non-zero and positive!";
			throw std::runtime_error(err_string);
		}
	}

	if(config_.module_positions.size() != 4){
		throw std::runtime_error(
			      "[SwerveModel::validateConfig] Error: module_positions must be of size four (one for each module).");
	}
}

void SwerveModel::calculateJacobians(){
	switch(config_.module_type){
		case swerve_type_e::COAXIAL:
		{
			module_jacobian_ << config_.wheel_ratio, 0.0, 0.0, config_.steering_ratio;
			module_jacobian_inv_ << 1 / config_.wheel_ratio, 0.0, 0.0, 1 / config_.steering_ratio;
		}
		break;

		case swerve_type_e::DIFFERENTIAL:
		{
			module_jacobian_ << config_.wheel_ratio / 2.0, -config_.wheel_ratio / 2.0, config_.steering_ratio / 2.0, config_.steering_ratio / 2.0;
			module_jacobian_inv_ << 1 / config_.wheel_ratio, 1 / config_.steering_ratio, -1 / config_.wheel_ratio, 1 / config_.steering_ratio;
		}
		break;
	}

	module_jacobian_transpose_ = module_jacobian_.transpose();
	module_jacobian_inv_transpose_ = module_jacobian_inv_.transpose();
}

void SwerveModel::calculateMaxBaseTwist(){
	// Get Max Base Speeds
	double max_wheel_dist = 0.0;
	for(const auto& [key, val] : config_.module_positions){
		max_wheel_dist = std::max(max_wheel_dist, (double) val.norm());
	}

	max_base_lin_vel_ = config_.max_wheel_lin_vel;
	max_base_ang_vel_ = max_base_lin_vel_ / max_wheel_dist;
}

void SwerveModel::calculateHSpaceICR(){
}

void SwerveModel::calculateOdometry(){
}

void SwerveModel::updateSwerveCommandsFromTwist(Eigen::Vector3f twist_cmd){
}

} // namespace ghost_swerve