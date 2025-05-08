#include "ghost_tank/bt_nodes/climbCmd.hpp"

namespace ghost_tank {

ClimbCmd::ClimbCmd(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
{
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "rhi_ptr", rhi_ptr_);
}

BT::PortsList ClimbCmd::providedPorts()
{
  return {
    BT::InputPort<bool>("climb_extended")
  };
}

BT::NodeStatus ClimbCmd::tick()
{
  bool climb_up = BT_Util::get_input<bool>(this, "climb_extended");
  BT_Util::put_in_blackboard(blackboard_, "climb_extended", climb_up);
  return BT::NodeStatus::SUCCESS;
}

} // namespace ghost_tank
