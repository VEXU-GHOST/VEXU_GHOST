#include <ghost_swerve/swerve_model.hpp>
#include <ghost_util/angle_util.hpp>

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
	calculateHSpaceICR();
}

void SwerveModel::updateSwerveCommandFromNormalizedTwist(Eigen::Vector3d twist_norm){
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

std::vector<Line2d> SwerveModel::calculateWheelAxisVectors() const {
	std::vector<Line2d> axis_vectors;

	// Convert each wheel module to Line2d
	for(const auto & [name, start_point] : m_config.module_positions){
		// Get angle of module in radians
		double angle_rad = (m_current_module_states.at(name).steering_position) * M_PI / 180.0;

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

Eigen::Vector3d SwerveModel::averageVectorAntipoles(std::vector<Eigen::Vector3d> vectors){
	if(vectors.size() == 0){
		throw std::runtime_error("[SwerveModel::averageVectorAntipoles] Error: vector list is empty!");
	}

	// Find fusion candidates
	auto first_vector = vectors[0].normalized();
	std::vector<Eigen::Vector3d> candidates{first_vector};
	for(int i = 1; i < vectors.size(); i++){
		auto p = vectors[i].normalized();
		auto d1 = (first_vector - p).norm();
		auto d2 = (first_vector - (-p)).norm();
		if(d1 < d2){
			candidates.push_back(p);
		}
		else{
			candidates.push_back(-p);
		}
	}

	Eigen::Vector3d avg(0, 0, 0);
	for(const auto &p : candidates){
		avg += p;
	}
	return (avg / candidates.size()).normalized();
}

void SwerveModel::calculateHSpaceICR(){
	std::vector<Line2d>  axis_vectors = calculateWheelAxisVectors();
	std::vector<Eigen::Vector3d> h_space_unit_vectors = calculateSphericalProjectionAxisIntersections(axis_vectors);
	filterCollinearVectors(h_space_unit_vectors, axis_vectors.size());
	m_h_space_projection = averageVectorAntipoles(h_space_unit_vectors);

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
	// If two axes are collinear but the robot isn't moving straight, it throws off the ICR average.
	// Thus, if there is only one collinear point, remove it.
	// Do it in reverse so we can erase later
	std::vector<int> collinear_point_indices;
	for(int i = vectors.size() - 1; i >= 0; i--){
		if(vectors[i][2] < 1e-9){
			collinear_point_indices.push_back(i);
		}
	}

	if(collinear_point_indices.size() <= num_modules / 2){
		for(const auto & index : collinear_point_indices){
			vectors.erase(vectors.begin() + index);
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

} // namespace ghost_swerve