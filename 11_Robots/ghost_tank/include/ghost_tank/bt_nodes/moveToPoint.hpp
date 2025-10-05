#include <math.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "ghost_tank/bt_nodes/bt_util.hpp"
#include "ghost_util/angle_util.hpp"
#include "ghost_util/unit_conversion_utils.hpp"

#include "rclcpp/rclcpp.hpp"


namespace ghost_tank
{
class MoveToPoint : public BT::StatefulActionNode
{

  //takes input for desired point
  //turns to face that point

public:
  MoveToPoint(const std::string & name, const BT::NodeConfig & config);
  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();
  /// Method called once, when transitioning from the state IDLE.
  /// If it returns RUNNING, this becomes an asynchronous node.
  BT::NodeStatus onStart();

  /// method invoked when the action is already in the RUNNING state.
  BT::NodeStatus onRunning();

  /// when the method halt() is called and the action is RUNNING, this method is invoked.
  /// This is a convenient place todo a cleanup, if needed.
  void onHalted();

private:
  std::shared_ptr<TankModel> tank_model_ptr_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  BT::Blackboard::Ptr blackboard_;

  bool first_loop_;
  double Kd{0.0};
  double Kp{0.0};
  int timeout_ms{0};
  double distance_m{0.0};
  Eigen::Vector2d start_position_;
  bool backwards{false};

  static constexpr double tile_to_meters = 0.6096;

  void move();
};
}
