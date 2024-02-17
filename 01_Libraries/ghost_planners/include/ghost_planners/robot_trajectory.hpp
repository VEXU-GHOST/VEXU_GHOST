#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
// #include "ghost_v5_interfaces/devices/motor_device_interface.hpp"

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

        // command type
        // bool position_command = false;
        // bool velocity_command = false;
        // bool voltage_command = false;
        // bool torque_command = false;

        std::tuple<bool, double> getPosition(double time){
            return interpolate(position_vector, time);
        }
        std::tuple<bool, double> getVelocity(double time){
            return interpolate(velocity_vector, time);
        }
        std::tuple<bool, double> getVoltage(double time){
            return interpolate(voltage_vector, time);
        }
        std::tuple<bool, double> getTorque(double time){
            return interpolate(torque_vector, time);
        }

        std::tuple<bool, double> interpolate(std::vector<double>& data, double time);

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
		    //    (trajectory_map == rhs.trajectory_map) && 
               (motor_trajectories == rhs.motor_trajectories);
	}
};

} // namespace ghost_planners