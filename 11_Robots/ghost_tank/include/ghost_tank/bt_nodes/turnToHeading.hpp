#include <math.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_tank/control/tank_pid_controller.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include <ghost_control/pid_controller.hpp>

namespace ghost_tank {

// Rotates the chassis in place to an absolute map-frame heading, using the same
// arc-turn PID controller as TurnToPoint. Unlike TurnToPoint (which turns to
// *face a point*), the target here is a fixed heading -- the natural pairing for
// settling onto GeneratePlannerPath's commanded goal yaw when the nav2
// controller is configured with use_rotate_to_heading=false and so never spins
// to the final heading itself.
class TurnToHeading : public BT::StatefulActionNode {

public:
    TurnToHeading(const std::string & name, const BT::NodeConfig & config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart();
    BT::NodeStatus onRunning();
    void onHalted();

private:
    std::shared_ptr<TankModel> tank_model_ptr_;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    BT::Blackboard::Ptr blackboard_;
    std::shared_ptr<ghost_control::PIDController> m_arc_turn_controller_ptr;

    int timeout_ms;
    double angle_exit_threshold_rad;
    double des_ang_rad;
    bool backwards;
};

} // namespace ghost_tank
