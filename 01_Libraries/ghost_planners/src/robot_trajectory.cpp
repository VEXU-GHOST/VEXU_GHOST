#include "ghost_planners/robot_trajectory.hpp"

namespace ghost_planners {

std::tuple<bool, double>  RobotTrajectory::MotorTrajectory::interpolate(std::vector<double>& data, double time){
    if (data.empty()){
        return {false, 0};
    }
    if (data.size() == 1){
        return {true, data[0]};
    }
    if (time >= time_vector[data.size() - 1]){
        return {true, data[data.size() - 1]};
    }
    int index = 0;
    auto iter = std::lower_bound(time_vector.begin(), time_vector.end(), time);
    if (iter == time_vector.end()){
        //not found in time
        return {true, data[data.size() - 1]};
    } else {
        index = iter - time_vector.begin();
    }

    double lowTime = time_vector[index];
    double highTime = time_vector[index + 1];
    double lowValue = data[index];
    double highValue = data[index + 1];
    double x_value = (time - lowTime)/(highTime - lowTime);
    return {true, x_value*(highValue - lowValue) + lowValue};
}

RobotTrajectory::RobotTrajectory(){
    motor_names = std::vector<std::string>();
    motor_trajectories = std::vector<MotorTrajectory>();
}

void RobotTrajectory::add_trajectory(std::shared_ptr<RobotTrajectory::MotorTrajectory> motor_trajectory_ptr){
    motor_trajectories.push_back(*motor_trajectory_ptr);
}


// // make function that takes time and returns interpolated motor device data
// RobotTrajectory::MotorTrajectory RobotTrajectory::get_motor_values(std::string motor_name, double time){
//     MotorTrajectory trajectory = trajectory_map[motor_name];

//     // does returning zero mean 0 or no command?
//     double position = trajectory.getPosition(time);
//     double velocity = trajectory.getVelocity(time);
//     double voltage = trajectory.getVoltage(time);
//     double torque = trajectory.getTorque(time);
// }

} // namespace ghost_planners
