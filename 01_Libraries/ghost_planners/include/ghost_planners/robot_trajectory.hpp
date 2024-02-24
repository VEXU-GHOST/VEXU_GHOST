#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "ghost_util/math_util.hpp"

namespace ghost_planners {

class RobotTrajectory {
public:

	struct MotorTrajectory {
		MotorTrajectory(){
			time_vector = std::vector<double>();
			position_vector = std::vector<double>();
			velocity_vector = std::vector<double>();
			voltage_vector = std::vector<double>();
			torque_vector = std::vector<double>();
		}

		std::tuple<bool, double> getPosition(double time) const {
			if(position_vector.size() == 0){
				return {false, 0};
			}
			return {true, ghost_util::clampedLinearInterpolate(time_vector, position_vector, time)};
		}
		std::tuple<bool, double> getVelocity(double time) const {
			if(velocity_vector.size() == 0){
				return {false, 0};
			}
			return {true, ghost_util::clampedLinearInterpolate(time_vector, velocity_vector, time)};
		}
		std::tuple<bool, double> getVoltage(double time) const {
			if(voltage_vector.size() == 0){
				return {false, 0};
			}
			return {true, ghost_util::clampedLinearInterpolate(time_vector, voltage_vector, time)};
		}
		std::tuple<bool, double> getTorque(double time) const {
			if(torque_vector.size() == 0){
				return {false, 0};
			}
			return {true, ghost_util::clampedLinearInterpolate(time_vector, torque_vector, time)};
		}

		// values
		std::vector<double> time_vector;
		std::vector<double> position_vector;
		std::vector<double> velocity_vector;
		std::vector<double> voltage_vector;
		std::vector<double> torque_vector;

		bool operator==(const RobotTrajectory::MotorTrajectory &rhs) const {
			return (time_vector == rhs.time_vector) && (position_vector == rhs.position_vector) &&
			       (velocity_vector == rhs.velocity_vector) && (voltage_vector == rhs.voltage_vector) &&
			       (torque_vector == rhs.torque_vector);
		}
	};

	RobotTrajectory();

	void add_trajectory(std::shared_ptr<MotorTrajectory> motor_trajectory_ptr);

	MotorTrajectory get_motor_values(std::string motor_name, double time);

	std::unordered_map<std::string, MotorTrajectory> trajectory_map;
	std::vector<std::string> motor_names;
	std::vector<MotorTrajectory> motor_trajectories;

	bool operator==(const RobotTrajectory &rhs) const {
		return (motor_names == rhs.motor_names) &&
		       (motor_trajectories == rhs.motor_trajectories);
	}
};

} // namespace ghost_planners