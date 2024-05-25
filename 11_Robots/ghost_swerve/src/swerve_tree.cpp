#include "ghost_swerve/swerve_tree.hpp"
#include <ghost_util/angle_util.hpp>


// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

namespace ghost_swerve {

SwerveTree::SwerveTree(std::string bt_path, 
						std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr,
						std::shared_ptr<SwerveModel> swerve_ptr,
						double burnout_absolute_rpm_threshold,
						double burnout_stall_duration_ms,
						double burnout_cooldown_duration_ms,
						double lift_setpoint) :
			bt_path_(bt_path)
		{

		BT::BehaviorTreeFactory factory;

		// add all nodes here
		factory.registerNodeType<LoggingNode>("Logging");
		factory.registerNodeType<CheckForRestart>("CheckForRestart", robot_hardware_interface_ptr);
		factory.registerNodeType<MoveToPose>("MoveToPose", robot_hardware_interface_ptr, swerve_ptr);
		factory.registerNodeType<SwipeTail>("SwipeTail", robot_hardware_interface_ptr, swerve_ptr);
		factory.registerNodeType<IntakeCmd>("IntakeCmd", robot_hardware_interface_ptr, swerve_ptr,
					burnout_absolute_rpm_threshold,
					burnout_stall_duration_ms,
					burnout_cooldown_duration_ms,
					lift_setpoint);
		factory.registerNodeType<Climb>("Climb", robot_hardware_interface_ptr, swerve_ptr);
		factory.registerNodeType<AutoDone>("AutoDone", robot_hardware_interface_ptr, swerve_ptr);

		tree_ = factory.createTreeFromFile(bt_path_);
}

void SwerveTree::tick_tree(){
		tree_.tickExactlyOnce();
}

} // namespace ghost_swerve