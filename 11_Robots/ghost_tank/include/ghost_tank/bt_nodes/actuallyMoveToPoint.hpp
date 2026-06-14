#include <math.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/control/tank_pid_controller.hpp"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"

#include <ghost_control/pid_controller.hpp>

namespace ghost_tank
{

// Takes input for a desired point, first turns to face that point, then drives
// straight to it. Combines TurnToPoint and MoveToPoint into a single node.
class ActuallyMoveToPoint : public BT::StatefulActionNode
{

public:
  ActuallyMoveToPoint(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();
  void onHalted();

private:
  enum class Phase { TURNING, DRIVING };

  void move();

  std::shared_ptr<TankModel> tank_model_ptr_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> drive_start_time_;
  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<ghost_control::PIDController> m_arc_turn_controller_ptr;
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr expected_pose_pub_;

  Phase phase_;

  double posX_m;
  double posY_m;
  int turn_timeout_ms;
  int move_timeout_ms;
  bool backwards;

  // turn phase
  double angle_exit_threshold_rad;
  double des_ang_rad;
  int angle_settle_ms;
  bool angle_settling_{false};
  std::chrono::time_point<std::chrono::system_clock> angle_settle_start_;

  // drive phase
  double Kp{0.0};
  double Kd{0.0};
  double distance_m{0.0};
  Eigen::Vector2d start_position_;
};

} // namespace ghost_tank
