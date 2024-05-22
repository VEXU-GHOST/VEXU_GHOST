#include <ghost_util/angle_util.hpp>
#include "ghost_swerve/swerve_tree.hpp"


// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

namespace ghost_swerve {

SwerveTree::SwerveTree(std::string bt_path,
                       std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr,
                       std::shared_ptr<SwerveModel> swerve_ptr,
                       std::shared_ptr<rclcpp::Node> node_ptr) :
	bt_path_(bt_path),
	node_ptr_(node_ptr){
	BT::BehaviorTreeFactory factory;

	// add all nodes here
	factory.registerNodeType<LoggingNode>("Logging", node_ptr_);
	factory.registerNodeType<CheckForRestart>("CheckForRestart", node_ptr_, robot_hardware_interface_ptr);
	factory.registerNodeType<MoveToPoseCubic>("MoveToPoseCubic", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);
	factory.registerNodeType<SwipeTail>("SwipeTail", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);
	factory.registerNodeType<IntakeCmd>("IntakeCmd", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);
	factory.registerNodeType<Climb>("Climb", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);
	// factory.registerNodeType<AutoDone>("AutoDone", node_ptr_, robot_hardware_interface_ptr, swerve_ptr);

	tree_ = factory.createTreeFromFile(bt_path_);
}

void SwerveTree::tick_tree(){
	tree_.tickExactlyOnce();
}

} // namespace ghost_swerve