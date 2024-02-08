#include "ghost_autonomy/bt_nodes/loggingNode.hpp"

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
SaySomething::SaySomething(const std::string& name, const BT::NodeConfig& config) :
		BT::SyncActionNode(name, config),
		rclcpp::Node("test_node"){
}

// It is mandatory to define this STATIC method.
BT::PortsList SaySomething::providedPorts(){
		// This action has a single input port called "message"
		return {
				BT::InputPort<std::string>("message")
		};
}

// Override the virtual function tick()
BT::NodeStatus SaySomething::tick() {
		BT::Expected<std::string> msg = getInput<std::string>("message");
		// Check if expected is valid. If not, throw its error
		if(!msg){
				throw BT::RuntimeError("missing required input [message]: ",
										msg.error() );
		}
		// use the method value() to extract the valid message.
		RCLCPP_INFO(this->get_logger(), msg.value().c_str());
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return BT::NodeStatus::SUCCESS;
}