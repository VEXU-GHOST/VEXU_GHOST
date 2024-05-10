#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "ghost_util/math_util.hpp"

namespace ghost_planners {

class RobotTrajectory {
public:

	struct Trajectory {
		Trajectory(){
			time_vector = std::vector<double>();
			position_vector = std::vector<double>();
			velocity_vector = std::vector<double>();

			threshold = 1.0;
			// voltage_vector = std::vector<double>();
			// torque_vector = std::vector<double>();
		}
		double getPosition(double time) const {
			return ghost_util::clampedLinearInterpolate(time_vector, position_vector, time);
		}
		double getVelocity(double time) const {
			if (!time_vector.empty() && time > *time_vector.end()){
				return 0.5 * *velocity_vector.end();
			}
			return ghost_util::clampedLinearInterpolate(time_vector, velocity_vector, time);
		}
		std::tuple<bool, double> getPositionAndCheck(double time) const {
			if(position_vector.size() == 0){
				return {false, 0};
			}
			return {true, ghost_util::clampedLinearInterpolate(time_vector, position_vector, time)};
		}
		std::tuple<bool, double> getVelocityAndCheck(double time) const {
			if(velocity_vector.size() == 0){
				return {false, 0};
			}
			return {true, ghost_util::clampedLinearInterpolate(time_vector, velocity_vector, time)};
		}
		// std::tuple<bool, double> getVoltage(double time) const {
		// 	if(voltage_vector.size() == 0){
		// 		return {false, 0};
		// 	}
		// 	return {true, ghost_util::clampedLinearInterpolate(time_vector, voltage_vector, time)};
		// }
		// std::tuple<bool, double> getTorque(double time) const {
		// 	if(torque_vector.size() == 0){
		// 		return {false, 0};
		// 	}
		// 	return {true, ghost_util::clampedLinearInterpolate(time_vector, torque_vector, time)};
		// }

		// values
		std::vector<double> time_vector;
		std::vector<double> position_vector;
		std::vector<double> velocity_vector;

		double threshold;
		// std::vector<double> voltage_vector;
		// std::vector<double> torque_vector;

		bool operator==(const RobotTrajectory::Trajectory &rhs) const {
			return (time_vector == rhs.time_vector) && (position_vector == rhs.position_vector) &&
			       (velocity_vector == rhs.velocity_vector);// && (voltage_vector == rhs.voltage_vector) &&
			    //    (torque_vector == rhs.torque_vector);
		}
	};

	RobotTrajectory();

	// Trajectory get_values(double time);

	Trajectory x_trajectory;
	Trajectory y_trajectory;
	Trajectory theta_trajectory;

	bool operator==(const RobotTrajectory &rhs) const {
		return (x_trajectory == rhs.x_trajectory) &&
			   (y_trajectory == rhs.y_trajectory) &&
			   (theta_trajectory == rhs.theta_trajectory);
	}

	operator bool() const { 
		return !x_trajectory.time_vector.empty() &&
			   !y_trajectory.time_vector.empty() &&
			   !theta_trajectory.time_vector.empty(); 
	}
};

} // namespace ghost_planners