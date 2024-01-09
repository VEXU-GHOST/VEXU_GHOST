#include "ghost_autonomy/run_tree.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

RunTreeNode::RunTreeNode(std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
		rclcpp::Node("run_tree_node"),
		robot_hardware_interface_ptr_(robot_hardware_interface_ptr){
		declare_parameter<std::string>("bt_path");
		bt_path_ = get_parameter("bt_path").as_string();
}

void RunTreeNode::run_tree(){
		// todo: add ros node to blackboard possibly
		// 		maybe not, seems unnecessary
		// todo: move this to another class an import
		BT::BehaviorTreeFactory factory;

		// add all nodes here
		factory.registerNodeType<SaySomething>("SaySomething");
		factory.registerNodeType<DriveForward>("DriveForward");

		auto tree = factory.createTreeFromFile(bt_path_);
		tree.rootBlackboard().get()->set<std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> >("robot_hardware_interface_ptr", robot_hardware_interface_ptr_);
		tree.tickWhileRunning();
}


int main(int argc, char *argv[]){
		rclcpp::init(argc, argv);
		auto node = std::make_shared<RunTreeNode>();
		rclcpp::spin(node);

		rclcpp::shutdown();
		return 0;
}