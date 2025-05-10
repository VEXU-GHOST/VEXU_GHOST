#include <math.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_tank/bezier_curve.hpp"
#include "ghost_tank/pdcontrol.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace ghost_tank {

class TurnToPoint : public BT::StatefulActionNode {

  // Takes input for desired point and turns to face that point

public:
    TurnToPoint(const std::string & name, const BT::NodeConfig & config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart();
    BT::NodeStatus onRunning();
    void onHalted();

private:
    std::shared_ptr<TankModel> tank_model_ptr_;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    BT::Blackboard::Ptr blackboard_;
    std::shared_ptr<PDControl> pd_control_ptr_;

    double posX_m;
    double posY_m;
    int timeout_ms;
    double angle_exit_threshold_rad;
    double des_ang_rad;

    static constexpr double tile_to_meters = 0.6096;
};

} // namespace ghost_tank
