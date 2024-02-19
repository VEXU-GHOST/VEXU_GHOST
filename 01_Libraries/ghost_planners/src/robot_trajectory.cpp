#include "ghost_planners/robot_trajectory.hpp"

namespace ghost_planners {

RobotTrajectory::RobotTrajectory(){
	motor_names = std::vector<std::string>();
	motor_trajectories = std::vector<MotorTrajectory>();
}

void RobotTrajectory::add_trajectory(std::shared_ptr<RobotTrajectory::MotorTrajectory> motor_trajectory_ptr){
	motor_trajectories.push_back(*motor_trajectory_ptr);
}

} // namespace ghost_planners