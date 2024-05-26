#include "ghost_swerve/bt_nodes/autoDone.hpp"

namespace ghost_swerve {

AutoDone::AutoDone(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr,
			std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
			std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::SyncActionNode(name, config),
	rclcpp::Node("intake_cmd"),
  	node_ptr_(node_ptr),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr){
}

// It is mandatory to define this STATIC method.
BT::PortsList AutoDone::providedPorts(){
	return {
	    // BT::InputPort<bool>("in"),
	};
}

template <typename T>
T AutoDone::get_input(std::string key){
	BT::Expected<T> input = getInput<T>(key);
	// Check if expected is valid. If not, throw its error
	if(!input){
		throw BT::RuntimeError("missing required input [" + key + "]: ",
		                       input.error() );
	}
	return input.value();
}

BT::NodeStatus AutoDone::tick() {
	auto status = BT::NodeStatus::FAILURE;

	swerve_ptr_->setAutoStatus(true);
	return status;
}


} // namespace ghost_swerve
