#include "ghost_autonomy/run_tree.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

RunTree::RunTree(std::string bt_path, std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
		// rclcpp::Node("run_tree_node")
			bt_path_(bt_path),
			robot_hardware_interface_ptr_(robot_hardware_interface_ptr)
		{
		// declare_parameter<std::string>("bt_path");
		// bt_path_ = get_parameter("bt_path").as_string();

		// declare_parameter<std::string>("bt_topic", "/behavior_tree/auton");
		// std::string bt_topic = get_parameter("bt_topic").as_string();

		// subscriber to run tree
		// bt_auton_sub_ = create_subscription<std_msgs::msg::Bool>(
		// 		bt_topic,
		// 		10,
		// 		std::bind(&RunTreeNode::btStartCallback, this, _1)
		// 		);
		BT::BehaviorTreeFactory factory;

		// add all nodes here
		factory.registerNodeType<SaySomething>("SaySomething");
		factory.registerNodeType<DriveForward>("DriveForward");

		tree_ = factory.createTreeFromFile(bt_path_);
		tree_.rootBlackboard().get()->set<std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> >("robot_hardware_interface_ptr", robot_hardware_interface_ptr_);

}

void RunTree::tick_tree(){
		tree_.tickExactlyOnce();
}

// void RunTree::btStartCallback(const std_msgs::msg::Bool::SharedPtr msg){
// 		if(msg->data){
// 				run_tree();
// 		}
// }

// int main(int argc, char *argv[]){
// 		rclcpp::init(argc, argv);
// 		auto node = std::make_shared<RunTreeNode>();
// 		rclcpp::spin(node);

// 		rclcpp::shutdown();
// 		return 0;
// }