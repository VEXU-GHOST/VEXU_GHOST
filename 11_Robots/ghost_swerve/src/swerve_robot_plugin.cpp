#include <ghost_swerve/swerve_robot_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <iostream>

namespace ghost_swerve {

SwerveRobotPlugin::SwerveRobotPlugin(){
}

void SwerveRobotPlugin::initialize(){
	std::cout << "Swerve Robot Initialization!" << std::endl;
}
void SwerveRobotPlugin::disabled(){
}
void SwerveRobotPlugin::autonomous(double current_time){
	std::cout << "Autonomous: " << current_time << std::endl;
}
void SwerveRobotPlugin::teleop(double current_time){
	std::cout << "Teleop: " << current_time << std::endl;
}

} // namespace ghost_swerve


PLUGINLIB_EXPORT_CLASS(ghost_swerve::SwerveRobotPlugin, ghost_ros_interfaces::V5RobotBase)