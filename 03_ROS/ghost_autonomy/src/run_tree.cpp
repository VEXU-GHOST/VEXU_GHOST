#include "ghost_autonomy/run_tree.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

RunTree::RunTree(std::string bt_path, std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
			bt_path_(bt_path)
		{

		BT::BehaviorTreeFactory factory;

		// add all nodes here
		factory.registerNodeType<SaySomething>("SaySomething");
		factory.registerNodeType<DriveForward>("DriveForward", robot_hardware_interface_ptr);
		factory.registerNodeType<MoveToPose>("MoveToPose", robot_hardware_interface_ptr);

		tree_ = factory.createTreeFromFile(bt_path_);
}

void RunTree::tick_tree(){
		tree_.tickExactlyOnce();
}
