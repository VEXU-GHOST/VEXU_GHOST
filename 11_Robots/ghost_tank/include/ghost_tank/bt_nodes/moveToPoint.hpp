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
  std::shared_ptr<PDControl> pd_control_ptr_;
  std::shared_ptr<PDControl> pd_control_threshold_ptr_;
  ghost_planners::RobotTrajectory robot_trajectory_;


  bool first_loop_;
  bool backwards;
  int past_index_;
  double search_radius{0.0};

  double forward_effort{0.0};
  double angular_effort{0.0};
  int timeout_ms{0};
  double posX_m{0.0};
  double posY_m{0.0};
  double cur_z{0.0};
  double des_ang{0.0};
  double max_speed_linear_percent{0.0};
  double max_speed_angular_percent{0.0};
  double angle_exit_threshold_rad{0.0};
  double xy_exit_threshold_m{0.0};

  static constexpr double tile_to_meters = 0.6096;

  void move();
};
}
