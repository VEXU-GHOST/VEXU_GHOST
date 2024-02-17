#include "ghost_planners/robot_trajectory.hpp"

namespace ghost_planners {

std::tuple<bool, double>  RobotTrajectory::MotorTrajectory::interpolate(std::vector<double>& data, double time) const {
	if(data.empty()){
		return {false, 0};
	}
	if(data.size() == 1){
		return {true, data[0]};
	}
	if(time >= time_vector[data.size() - 1]){
		return {true, time_vector[data.size() - 1]};
	}
	int index = 0;
	while(time < time_vector[index]){
		index++;
	}

	double lowTime = time_vector[index];
	double highTime = time_vector[index + 1];
	double lowValue = data[index];
	double highValue = data[index + 1];
	double x_value = (time - lowTime) / (highTime - lowTime);
	return {true, x_value*(highValue - lowValue) + lowValue};
}

RobotTrajectory::RobotTrajectory(){
	motor_names = std::vector<std::string>();
	motor_trajectories = std::vector<MotorTrajectory>();
}

void RobotTrajectory::add_trajectory(std::shared_ptr<RobotTrajectory::MotorTrajectory> motor_trajectory_ptr){
	motor_trajectories.push_back(*motor_trajectory_ptr);
}

} // namespace ghost_planners