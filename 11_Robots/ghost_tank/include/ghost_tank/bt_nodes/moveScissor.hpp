#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_tank/tank_model.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

using std::placeholders::_1;

namespace ghost_tank
{

class MoveScissor : public BT::StatefulActionNode
{
public:
    MoveScissor(const std::string & name, const BT::NodeConfig & config);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart();
    BT::NodeStatus onRunning();
    void onHalted();

private:
    std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_; // renamed from tank_model_ptr_
    BT::Blackboard::Ptr blackboard_;
    bool first_loop_;
    std::chrono::time_point<std::chrono::system_clock> start_time_;

    double target_position{0.0};
    double offset{0.0};
    int timeout_ms{0};
};

} // namespace ghost_tank
