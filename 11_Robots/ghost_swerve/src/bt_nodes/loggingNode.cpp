#include "ghost_swerve/bt_nodes/loggingNode.hpp"

// SyncActionNode (synchronous action) with an input port.
// If your Node has ports, you must use this constructor signature
LoggingNode::LoggingNode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr) :
		BT::SyncActionNode(name, config),
		node_ptr_(node_ptr){
}

// It is mandatory to define this STATIC method.
BT::PortsList LoggingNode::providedPorts(){
		// This action has a single input port called "message"
		return {
				BT::InputPort<std::string>("message")
		};
}

// Override the virtual function tick()
BT::NodeStatus LoggingNode::tick() {
		BT::Expected<std::string> msg = getInput<std::string>("message");
		// Check if expected is valid. If not, throw its error
		if(!msg){
				throw BT::RuntimeError("missing required input [message]: ",
										msg.error() );
		}
		// use the method value() to extract the valid message.
		RCLCPP_INFO(node_ptr_->get_logger(), msg.value().c_str());
		// std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return BT::NodeStatus::SUCCESS;
}