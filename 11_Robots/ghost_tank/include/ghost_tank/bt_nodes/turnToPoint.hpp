#include <math.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_msgs/msg/robot_trajectory.hpp"
#include "ghost_tank/control/tank_pid_controller.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"
#include "ghost_v5_interfaces/robot_hardware_interface.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <ghost_control/pid_controller.hpp>

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
    std::shared_ptr<ghost_control::PIDController> m_arc_turn_controller_ptr;
    std::shared_ptr<rclcpp::Node> node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr expected_pose_pub_;

    double posX_m;
    double posY_m;
    int timeout_ms;
    double angle_exit_threshold_rad;
    double des_ang_rad;
    bool backwards;
    int angle_settle_ms;
    bool angle_settling_{false};
    std::chrono::time_point<std::chrono::system_clock> angle_settle_start_;
};

} // namespace ghost_tank
