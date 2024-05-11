#include "ghost_swerve/bt_nodes/checkForRestart.hpp"

using std::placeholders::_1;

namespace ghost_swerve {

// If your Node has ports, you must use this constructor signature
CheckForRestart::CheckForRestart(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr,
                                 std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
	BT::SyncActionNode(name, config),
	node_ptr_(node_ptr),
	robot_hardware_interface_ptr_(robot_hardware_interface_ptr){
	start_time_ = std::chrono::system_clock::now();
	restarted_ = true;
}

// It is mandatory to define this STATIC method.
BT::PortsList CheckForRestart::providedPorts(){
	return {
	};
}

// Override the virtual function tick()
BT::NodeStatus CheckForRestart::tick() {
	if(restarted_){
		restarted_ = false;
		start_time_ = std::chrono::system_clock::now();
	}

	auto now = std::chrono::system_clock::now();
	double time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
	if(time_elapsed > 500){
		RCLCPP_INFO(node_ptr_->get_logger(), "BT reset");
		restarted_ = true;
		return BT::NodeStatus::FAILURE;
	}

	start_time_ = std::chrono::system_clock::now();

	return BT::NodeStatus::SUCCESS;
}

} // namespace ghost_swerve