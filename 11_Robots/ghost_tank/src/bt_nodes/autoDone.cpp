#include "ghost_tank/bt_nodes/autoDone.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"

namespace ghost_tank {

AutoDone::AutoDone(const std::string& name, const BT::NodeConfig& config) :
	BT::SyncActionNode(name, config){
    blackboard_ = config.blackboard;
	blackboard_->get("tank_model_ptr", tank_ptr_);
	blackboard_->get("node_ptr", node_ptr_);
}

// It is mandatory to define this STATIC method.
BT::PortsList AutoDone::providedPorts(){
	return {
	    // BT::InputPort<bool>("in"),
	};
}

BT::NodeStatus AutoDone::tick() {
	auto status = BT::NodeStatus::FAILURE;

	tank_ptr_->setAutoStatus(true);
	return status;
}


} // namespace ghost_tank
