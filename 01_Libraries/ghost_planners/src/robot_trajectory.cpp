#include "ghost_planners/robot_trajectory.hpp"

namespace ghost_planners {

RobotTrajectory::RobotTrajectory(){
	x_trajectory = Trajectory();
	y_trajectory = Trajectory();
	theta_trajectory = Trajectory();
}

} // namespace ghost_planners