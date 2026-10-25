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
// to it. Combines TurnToPoint and MoveForward into a single node. The drive phase
// uses the remaining distance (scaled by cos of the heading error) and holds
// heading toward the target until within heading_hold_cutoff_m. With chain=true
// the node keeps a minimum drive speed and doesn't stop at the end, so the next
// motion can start from the momentum.
class MoveToPoint : public BT::StatefulActionNode
{

public:
  MoveToPoint(const std::string & name, const BT::NodeConfig & config);

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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr starting_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr turn_direction_pub_;

  void publishPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
    double x, double y, double yaw);

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
  Eigen::Vector2d travel_dir_{Eigen::Vector2d::Zero()};
  double heading_hold_cutoff_m{0.2};
  bool heading_hold_active_{false};
  std::shared_ptr<ghost_control::PIDController> heading_controller_ptr_;

  // chaining
  bool chain_{false};
  double chain_min_effort_percent_{0.2};
  bool turned_{false};
};

} // namespace ghost_tank
