#include "ghost_tank/bt_nodes/arcturnToPoint.hpp"
#include <ghost_tank/visualization/visualization_helpers.hpp>
#include "ghost_util/angle_util.hpp"
#include <cmath>

namespace ghost_tank
{

ArcturnToPoint::ArcturnToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[ArcturnToPoint::ArcturnToPoint]" << std::endl;
  blackboard_ = config.blackboard;
  BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);
  BT_Util::get_from_blackboard(blackboard_, "arc_turn_controller_ptr", m_arc_turn_controller_ptr);
  BT_Util::get_from_blackboard(blackboard_, "node_ptr", node_ptr_);

  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg") * ghost_util::DEG_TO_RAD;
  ang_vel_exit_threshold_rps = BT_Util::get_input<double>(this, "ang_vel_exit_threshold_dps") * ghost_util::DEG_TO_RAD;
  drive_backwards = BT_Util::get_input<bool>(this, "drive_backwards");
  face_backwards = BT_Util::get_input<bool>(this, "face_backwards");

  path_viz_pub_ptr_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("/autonomy/follow_path/viz_markers", 10);

}

BT::PortsList ArcturnToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<int>("timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<double>("ang_vel_exit_threshold_dps"),
    BT::InputPort<bool>("face_backwards"),
    BT::InputPort<bool>("drive_backwards")
  };
}

BT::NodeStatus ArcturnToPoint::onStart()
{
  m_arc_turn_controller_ptr->reset();

  bool mirrored;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);
  start_time_ = std::chrono::system_clock::now();
  m_arc_turn_controller_ptr->reset();
  if (mirrored) {
    posX_m = 6.0 * ghost_util::TILES_TO_METERS - posX_m;
  }

  calculateDesiredAngle();

  double init_error = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool turn_right = init_error > 0.0;

  if (drive_backwards) {
    use_right_side = !turn_right;
  } else {
    use_right_side = turn_right;
  }

  return BT::NodeStatus::RUNNING;
}

void ArcturnToPoint::calculateDesiredAngle()
{
  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);

  if (face_backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }
}

BT::NodeStatus ArcturnToPoint::onRunning()
{
  calculateDesiredAngle();

  double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
  bool angle_satisfied = std::fabs(theta_err_rad) < angle_exit_threshold_rad;
  bool ang_vel_satisfied = std::fabs(tank_model_ptr_->getWorldTwist().z()) < ang_vel_exit_threshold_rps;

  std::cout << "theta error: " << theta_err_rad << std::endl;

  int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
  if ((angle_satisfied && ang_vel_satisfied) || time_elapsed > timeout_ms) {
    tank_model_ptr_->driveCommandTank(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  auto command = m_arc_turn_controller_ptr->calculateCommand(theta_err_rad, -tank_model_ptr_->getWorldTwist().z());

  double cmd_sign = (command > 0.0) ? 1.0 : -1.0;
  cmd_sign = (std::fabs(command) > 0.01) ? cmd_sign : 0.0;
  double breaking = 0.01 * cmd_sign;

  if (use_right_side) {
    std::cout << "use_right" << command << std::endl;
    tank_model_ptr_->driveCommandTank(-breaking, command);
  } else {
    std::cout << "use_left: " << -command << std::endl;
    tank_model_ptr_->driveCommandTank(-command, breaking);
  }

  std::cout << std::endl;
  visualization();

  return BT::NodeStatus::RUNNING;

}
void ArcturnToPoint::onHalted()
{
  resetStatus();
}

void ArcturnToPoint::visualization()
{
  viz_msg_.markers.clear();

  Eigen::Vector2d target(posX_m, posY_m);
  visualization::getLineMarker(
    viz_msg_,
    tank_model_ptr_->getWorldPose().head<2>(),
    target,
    0.25 * visualization::MARKER_Z_OFFSET,
    visualization::getColorRGBA(0.0, 1.0, 0.0, 1.0)
  );

  path_viz_pub_ptr_->publish(viz_msg_);

}

} // namespace ghost_tank
