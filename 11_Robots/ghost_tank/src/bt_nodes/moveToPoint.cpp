#include "ghost_tank/bt_nodes/moveToPoint.hpp"
#include <cmath>

namespace ghost_tank
{

MoveToPoint::MoveToPoint(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  std::cout << "[MoveToPoint::MoveToPoint]" << std::endl;

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
}

BT::PortsList MoveToPoint::providedPorts()
{
  return {
    BT::InputPort<double>("posX_tiles"),
    BT::InputPort<double>("posY_tiles"),
    BT::InputPort<bool>("backwards"),
    BT::InputPort<int>("turn_timeout_ms"),
    BT::InputPort<int>("move_timeout_ms"),
    BT::InputPort<double>("angle_exit_threshold_deg"),
    BT::InputPort<int>("angle_settle_ms"),
    BT::InputPort<double>("kp"),
    BT::InputPort<double>("kd"),
    BT::InputPort<double>("max_effort_percent"),
    BT::InputPort<double>("heading_hold_cutoff_m"),
    BT::InputPort<double>("heading_kp"),
    BT::InputPort<double>("heading_kd"),
    BT::InputPort<bool>("chain"),
    BT::InputPort<double>("chain_min_effort_percent"),
    BT::InputPort<double>("settle_dist_m"),
    BT::InputPort<double>("settle_speed_mps"),
    BT::InputPort<double>("ang_vel_exit_threshold_dps")
  };
}

BT::NodeStatus MoveToPoint::onStart()
{
  posX_m = BT_Util::get_input<double>(this, "posX_tiles") * ghost_util::TILES_TO_METERS;
  posY_m = BT_Util::get_input<double>(this, "posY_tiles") * ghost_util::TILES_TO_METERS;
  turn_timeout_ms = BT_Util::get_input<int>(this, "turn_timeout_ms");
  move_timeout_ms = BT_Util::get_input<int>(this, "move_timeout_ms");
  backwards = BT_Util::get_input<bool>(this, "backwards", false);
  angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
  angle_settle_ms = BT_Util::get_input<int>(this, "angle_settle_ms", 0);
  heading_hold_cutoff_m = BT_Util::get_input<double>(this, "heading_hold_cutoff_m", 0.2);
  max_effort_percent_ = BT_Util::get_input<double>(this, "max_effort_percent");
  Kp = BT_Util::get_input<double>(this, "kp");
  Kd = BT_Util::get_input<double>(this, "kd");
  settle_dist_m_ = BT_Util::get_input<double>(this, "settle_dist_m", 0.03);
  settle_speed_mps_ = BT_Util::get_input<double>(this, "settle_speed_mps", 0.05);
  ang_vel_exit_threshold_radps_ =
    BT_Util::get_input<double>(this, "ang_vel_exit_threshold_dps", 20.0) * ghost_util::DEG_TO_RAD;
  chain_ = BT_Util::get_input<bool>(this, "chain", false);
  chain_min_effort_percent_ = BT_Util::get_input<double>(this, "chain_min_effort_percent", 0.2);

  // Drive-phase heading hold uses its own gains when given, otherwise the
  // in-place turn controller.
  turned_ = false;
  if (getInput<double>("heading_kp") || getInput<double>("heading_kd")) {
    ghost_control::PIDConfig heading_config;
    heading_config.kp = BT_Util::get_input<double>(this, "heading_kp", 0.0);
    heading_config.kd = BT_Util::get_input<double>(this, "heading_kd", 0.0);
    heading_controller_ptr_ = std::make_shared<ghost_control::PIDController>(heading_config);
  } else {
    heading_controller_ptr_ = m_arc_turn_controller_ptr;
  }

  bool mirrored = false;
  BT_Util::get_from_blackboard(blackboard_, "mirrored", mirrored);
  if (mirrored) {
    posX_m = 6.0 * ghost_util::TILES_TO_METERS - posX_m;
  }

  double cur_x = tank_model_ptr_->getWorldPose().x();
  double cur_y = tank_model_ptr_->getWorldPose().y();

  start_time_ = std::chrono::steady_clock::now();
  des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x);
  if (backwards) {
    des_ang_rad = ghost_util::FlipAnglePI(des_ang_rad);
  }

  m_arc_turn_controller_ptr->reset();
  phase_ = Phase::TURNING;
  angle_settling_ = false;

  // Publish the heading target for the turn phase: the robot does not translate
  // while turning, so the expected pose keeps the current position.
  publishPose(expected_pose_pub_, cur_x, cur_y, des_ang_rad);

  // Starting pose is where the robot is now; target pose is the final point,
  // oriented along the travel heading.
  publishPose(starting_pose_pub_, cur_x, cur_y, tank_model_ptr_->getWorldPose().z());
  publishPose(target_pose_pub_, posX_m, posY_m, des_ang_rad);

  // Arrow from the robot pointing straight at the target point, ignoring the
  // backwards flip, so the turn target is visible regardless of drive direction.
  publishPose(turn_direction_pub_, cur_x, cur_y, std::atan2(posY_m - cur_y, posX_m - cur_x));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPoint::onRunning()
{
  if (phase_ == Phase::TURNING) {
    double theta_err_rad = ghost_util::SmallestAngleDistRad(des_ang_rad, tank_model_ptr_->getWorldPose().z());
    // Also require the robot to have stopped rotating, so we don't hand off to
    // the drive phase while still swinging through the target heading.
    bool angle_in_threshold = std::fabs(theta_err_rad) < angle_exit_threshold_rad &&
      std::fabs(tank_model_ptr_->getWorldTwist().z()) < ang_vel_exit_threshold_radps_;

    // Require the heading to stay within threshold for angle_settle_ms before
    // committing to the drive phase, so a momentary pass-through on overshoot
    // doesn't end the turn early. angle_settle_ms == 0 keeps the instantaneous
    // behaviour.
    bool angle_satisfied = false;
    if (angle_in_threshold) {
      if (!angle_settling_) {
        angle_settling_ = true;
        angle_settle_start_ = std::chrono::steady_clock::now();
      }
      int settled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - angle_settle_start_).count();
      angle_satisfied = settled_ms >= angle_settle_ms;
    } else {
      angle_settling_ = false;
    }

    int turn_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time_).count();
    bool turn_timed_out = turn_elapsed > turn_timeout_ms;

    // Transition to the drive phase once aimed, or once the turn times out
    // (drive anyway from the best heading we managed to reach).
    if (angle_satisfied || turn_timed_out) {
      if (turn_timed_out && !angle_satisfied) {
        RCLCPP_WARN(
          node_ptr_->get_logger(), "MoveToPoint: turn timed out after %d ms, heading error %.1f deg",
          turn_elapsed, theta_err_rad / ghost_util::DEG_TO_RAD);
      }
      // Done turning: capture the drive start position and the distance to the
      // target, then publish the position target for the drive phase.
      // Only brake if we actually turned; when already aimed (e.g. chained from a
      // previous move) keep the momentum we arrived with.
      if (turned_) {
        tank_model_ptr_->driveCommandArcade(0.0, 0.0);
      }
      start_position_ = tank_model_ptr_->getWorldPose().head<2>();
      Eigen::Vector2d start_to_target = Eigen::Vector2d(posX_m, posY_m) - start_position_;
      distance_m = start_to_target.norm();
      travel_dir_ = (distance_m > 1e-6) ? Eigen::Vector2d(start_to_target / distance_m) : Eigen::Vector2d::Zero();
      heading_hold_active_ = true;
      heading_controller_ptr_->reset();
      phase_ = Phase::DRIVING;
      drive_start_time_ = std::chrono::steady_clock::now();

      publishPose(expected_pose_pub_, posX_m, posY_m, des_ang_rad);
      return BT::NodeStatus::RUNNING;
    }

    turned_ = true;
    tank_model_ptr_->driveCommandArcade(
      0.0, m_arc_turn_controller_ptr->calculateCommand(theta_err_rad, -tank_model_ptr_->getWorldTwist().z()));
    return BT::NodeStatus::RUNNING;
  }

  // Phase::DRIVING
  // Done once the robot crosses the line through the target perpendicular to the
  // travel direction, so drift off the path can't make it overshoot or stop short.
  Eigen::Vector2d current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  Eigen::Vector2d remaining = Eigen::Vector2d(posX_m, posY_m) - current_position_;
  bool crossed_line = remaining.dot(travel_dir_) <= 0.0;

  // The P term fades to zero near the target, so a robot that stalls just short
  // of the exit line would otherwise wait out the whole timeout.
  bool settled = remaining.norm() < settle_dist_m_ &&
    tank_model_ptr_->getWorldTwist().head<2>().norm() < settle_speed_mps_;

  int drive_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - drive_start_time_).count();
  if (crossed_line) {
    return finishMove("crossed exit line");
  }
  if (settled) {
    return finishMove("settled");
  }
  if (drive_elapsed > move_timeout_ms) {
    return finishMove("TIMEOUT");
  }

  move();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPoint::finishMove(const char * reason)
{
  Eigen::Vector2d to_target = Eigen::Vector2d(posX_m, posY_m) - tank_model_ptr_->getWorldPose().head<2>();
  double bearing_rad = std::atan2(to_target.y(), to_target.x());
  if (backwards) {
    bearing_rad = ghost_util::FlipAnglePI(bearing_rad);
  }
  double heading_err_rad = ghost_util::SmallestAngleDistRad(bearing_rad, tank_model_ptr_->getWorldPose().z());
  int drive_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - drive_start_time_).count();

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "MoveToPoint: done (%s) after %d ms, dist to target %.3f m (along path %.3f m), heading error %.1f deg",
    reason, drive_elapsed, to_target.norm(), to_target.dot(travel_dir_),
    heading_err_rad / ghost_util::DEG_TO_RAD);

  // When chaining, leave the last command running; the next node overwrites it
  // on its first tick so the robot flows through the point without stopping.
  if (!chain_) {
    tank_model_ptr_->driveCommandArcade(0.0, 0.0);
  }
  return BT::NodeStatus::SUCCESS;
}

void MoveToPoint::move()
{
  Eigen::Vector2d current_position_ = tank_model_ptr_->getWorldPose().head<2>();
  Eigen::Vector2d to_target = Eigen::Vector2d(posX_m, posY_m) - current_position_;
  double dist_err = to_target.norm();

  // Re-aim at the target every cycle so drift or a rough turn is corrected.
  double bearing_rad = std::atan2(to_target.y(), to_target.x());
  if (backwards) {
    bearing_rad = ghost_util::FlipAnglePI(bearing_rad);
  }
  double heading_err_rad = ghost_util::SmallestAngleDistRad(bearing_rad, tank_model_ptr_->getWorldPose().z());

  // Project the drive command onto the line to the target: a large heading error
  // slows the forward command so the robot turns before it drives hard.
  double fwd_cmd = dist_err * Kp - (tank_model_ptr_->getWorldTwist().head<2>().norm()) * Kd;
  fwd_cmd *= std::max(0.0, std::cos(heading_err_rad));
  // When chaining, keep a minimum speed so the robot reaches the exit line with
  // momentum instead of decelerating to a stop short of it.
  if (chain_) {
    fwd_cmd = std::max(fwd_cmd, chain_min_effort_percent_);
  }
  fwd_cmd = ghost_util::clamp(fwd_cmd, -max_effort_percent_, max_effort_percent_);

  // Heading hold, latched off near the target where the bearing becomes noisy
  // and the robot should just coast in.
  if (dist_err < heading_hold_cutoff_m) {
    heading_hold_active_ = false;
  }
  double turn_cmd = 0.0;
  if (heading_hold_active_) {
    turn_cmd = heading_controller_ptr_->calculateCommand(
      heading_err_rad, -tank_model_ptr_->getWorldTwist().z());
    turn_cmd = ghost_util::clamp(turn_cmd, -max_effort_percent_, max_effort_percent_);
  }

  // driveCommandArcade clamps each wheel separately, which distorts the steering
  // ratio when fwd + turn exceeds full output. Scale both down together instead.
  double normalizer = 1.0 / std::max(1.0, std::fabs(fwd_cmd) + std::fabs(turn_cmd));
  fwd_cmd *= normalizer;
  turn_cmd *= normalizer;

  if (backwards) {
    fwd_cmd *= -1.0;
  }

  BT_Util::put_in_blackboard(blackboard_, "fwd_cmd", fwd_cmd);
  tank_model_ptr_->driveCommandArcade(fwd_cmd, turn_cmd);
}

void MoveToPoint::publishPose(
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

void MoveToPoint::onHalted()
{
  resetStatus();
}

} // namespace ghost_tank
