#include "alpha_jerry/bt_nodes/autoDone.hpp"
#include "alpha_jerry/bt_nodes/bt_util.hpp"

namespace alpha_jerry {

AutoDone::AutoDone(const std::string& name, const BT::NodeConfig& config) :
	BT::SyncActionNode(name, config){
    blackboard_ = config.blackboard;
	if(!blackboard_->get("node_ptr", node_ptr_)){
        std::cout << name << ": node_ptr not found in blackboard" << std::endl;
    }
}

// It is mandatory to define this STATIC method.
BT::PortsList AutoDone::providedPorts(){
	return {
	};
}

BT::NodeStatus AutoDone::tick() {
	auto status = BT::NodeStatus::FAILURE;
	return status;
}


} // namespace alpha_jerry
