#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/angle_util.hpp>

using geometry::Line;
using geometry::Line2f;

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
	state.steering_position = ghost_util::WrapAngle360(state.steering_position);
	m_previous_module_states[name] = m_current_module_states[name];
	m_current_module_states[name] = state;
}

void SwerveModel::updateSwerveModel(){
}

/*
   void GhostEstimatorNode::CalculateHSpaceICR(ghost_msgs::msg::V5SensorUpdate::SharedPtr encoder_msg){
        // Calculate Odometry
        auto left_encoder = encoder_msg->encoders[ghost_v5_config::STEERING_LEFT_ENCODER];
        auto right_encoder = encoder_msg->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER];
        auto back_encoder = encoder_msg->encoders[ghost_v5_config::STEERING_BACK_ENCODER];

        // Calculate ICR
        std::vector<Eigen::Vector3f> h_space_icr_points{0};

        // Calculate Wheel Axis Unit Direction Vectors
        Eigen::Vector2f left_encoder_dir(
                sin(ghost_common::WrapAngle360(left_encoder.position_degrees) * M_PI / 180.0),
                -cos(ghost_common::WrapAngle360(left_encoder.position_degrees) * M_PI / 180.0));
        Eigen::Vector2f right_encoder_dir(
                sin(ghost_common::WrapAngle360(right_encoder.position_degrees) * M_PI / 180.0),
                -cos(ghost_common::WrapAngle360(right_encoder.position_degrees) * M_PI / 180.0));
        Eigen::Vector2f back_encoder_dir(
                sin(ghost_common::WrapAngle360(back_encoder.position_degrees) * M_PI / 180.0),
                -cos(ghost_common::WrapAngle360(back_encoder.position_degrees) * M_PI / 180.0));

        // Calculate Wheel Axis Vectors
        geometry::Line2f left_encoder_vector(left_wheel_link_, left_encoder_dir + left_wheel_link_);
        geometry::Line2f right_encoder_vector(right_wheel_link_, right_encoder_dir + right_wheel_link_);
        geometry::Line2f back_encoder_vector(back_wheel_link_, back_encoder_dir + back_wheel_link_);

        // Iterate through each pair of lines and calculate ICR
        auto line_pairs = std::vector<std::pair<geometry::Line2f, geometry::Line2f> >{
                std::pair<geometry::Line2f, geometry::Line2f>(left_encoder_vector, right_encoder_vector),
                std::pair<geometry::Line2f, geometry::Line2f>(back_encoder_vector, left_encoder_vector),
                std::pair<geometry::Line2f, geometry::Line2f>(back_encoder_vector, right_encoder_vector)
        };

        for(auto &pair : line_pairs){
                auto l1 = pair.first;
                auto l2 = pair.second;
                if(fabs(geometry::Cross(l1.Dir(), l2.Dir())) < 1e-5){
                        h_space_icr_points.push_back(Eigen::Vector3f(l1.Dir().x(), l1.Dir().y(), 0.0));
                }
                else{
                        Eigen::Hyperplane<float, 2> hp1 = Eigen::Hyperplane<float, 2>::Through(l1.p0, l1.p1);
                        Eigen::Hyperplane<float, 2> hp2 = Eigen::Hyperplane<float, 2>::Through(l2.p0, l2.p1);
                        auto intersection = hp1.intersection(hp2);
                        auto intersection_3d = Eigen::Vector3f(intersection.x(), intersection.y(), 1);
                        h_space_icr_points.push_back(intersection_3d / intersection_3d.norm());
                }
        }

        // Calculate distance from first point and subsequent points and their antipoles
        // Select closer of the two (point / antipole) for calculating average
        if((h_space_icr_points[0] - h_space_icr_points[1]).norm() > (h_space_icr_points[0] + h_space_icr_points[1]).norm()){
                h_space_icr_points[1] *= -1;
        }

        if((h_space_icr_points[0] - h_space_icr_points[2]).norm() > (h_space_icr_points[0] + h_space_icr_points[2]).norm()){
                h_space_icr_points[2] *= -1;
        }

        // Average ICR points in H-Space as our estimated center of rotation
        h_space_icr_avg_ = (h_space_icr_points[0] + h_space_icr_points[1] + h_space_icr_points[2]) / 3;

        // Handle parallel case
        if(fabs(h_space_icr_avg_[2]) < 1e-9){
                icr_flat_estimation_ = Eigen::Vector3f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0);
        }
        else{
                icr_flat_estimation_ = Eigen::Vector3f(h_space_icr_avg_[0] / h_space_icr_avg_[2], h_space_icr_avg_[1] / h_space_icr_avg_[2], 0);
        }

        // Initialize Visualization msg, and add debug visualization
        std::vector<geometry::Line2f> lines{left_encoder_vector, right_encoder_vector, back_encoder_vector};
        DrawWheelAxisVectors(lines);

        std::vector<Eigen::Vector3f> points{
                h_space_icr_points[0],
                h_space_icr_points[1],
                h_space_icr_points[2],
                h_space_icr_avg_,
                icr_flat_estimation_
        };
        DrawICRPoints(points);
   }
 */

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