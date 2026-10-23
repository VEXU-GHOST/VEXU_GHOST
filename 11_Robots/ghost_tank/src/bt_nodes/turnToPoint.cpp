#include "ghost_tank/bt_nodes/turnToPoint.hpp"
#include <cmath>

namespace ghost_tank
{

TurnToPoint::TurnToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[TurnToPoint::TurnToPoint]" << std::endl;

  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);
  if (node_ptr_) {
    expected_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/expectedpose", 10);
    target_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
    starting_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/starting_pose", 10);
    turn_direction_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/turn_direction", 10);
  }

  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  backwards = BT_Util::get_input<bool>(this, "backwards");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
  angle_settle_ms = BT_Util::get_input<int>(this, "angle_settle_ms", 0);
}

BT::PortsList TurnToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<bool>("backwards"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<int>("angle_settle_ms")
  };
}

BT::NodeStatus TurnToPoint::onStart()
{
  bool mirrored = false;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);

  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  if (mirrored) {
    posX_m = 6.0 * ghost_util::TILES_TO_METERS - posX_m;
  }

  start_time_ = std::chrono::system_clock::now();
  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);

  if (backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  m_arc_turn_controller_ptr->reset();
  angle_settling_ = false;

  // Publish the target the controller is driving toward: the robot does not
  // translate during a turn, so the expected pose keeps the current position
  // and adopts the desired heading.
  publishPose(expected_pose_pub_, cur_x, cur_y, des_ang_rad);

  // Starting pose is where the robot is now; target pose is the point being
  // turned toward, oriented along the desired heading.
  publishPose(starting_pose_pub_, cur_x, cur_y, tank_model_ptr_->getWorldPose().z());
  publishPose(target_pose_pub_, posX_m, posY_m, des_ang_rad);

  // Arrow from the robot pointing straight at the target point, ignoring the
  // backwards flip, so the turn target is visible regardless of drive direction.
  publishPose(turn_direction_pub_, cur_x, cur_y, std::atan2(posY_m - cur_y, posX_m - cur_x));

  return BT::NodeStatus::RUNNING;
}

void TurnToPoint::publishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
  double x, double y, double yaw)
{
  if (!pub) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_ptr_->get_clock()->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  pub->publish(pose);
}

BT::NodeStatus TurnToPoint::onRunning()
{
  double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool angle_in_threshold = std::fabs(theta_err_rad) < angle_exit_threshold_rad;

  // Require the heading to stay within threshold for angle_settle_ms before
  // exiting, so a momentary pass-through on overshoot doesn't end the turn
  // early. angle_settle_ms == 0 keeps the instantaneous behaviour.
  bool angle_satisfied = false;
  if (angle_in_threshold) {
    if (!angle_settling_) {
      angle_settling_ = true;
      angle_settle_start_ = std::chrono::system_clock::now();
    }
    int settled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - angle_settle_start_).count();
    angle_satisfied = settled_ms >= angle_settle_ms;
  } else {
    angle_settling_ = false;
  }

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if (angle_satisfied || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  tank_model_ptr_->driveCommandArcade(0.0, m_arc_turn_controller_ptr->calculateCommand(theta_err_rad, -tank_model_ptr_->getWorldTwist().z()));
  return BT::NodeStatus::RUNNING;
}

void TurnToPoint::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
