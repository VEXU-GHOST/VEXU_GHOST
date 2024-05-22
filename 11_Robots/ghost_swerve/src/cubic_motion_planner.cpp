#include "ghost_swerve/cubic_motion_planner.hpp"

namespace ghost_swerve
{

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using std::placeholders::_1;

void CubicMotionPlanner::initialize()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "initializing");
}


void CubicMotionPlanner::generateMotionPlan(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Generating Swerve Motion Plan");

  double theta_f = ghost_util::quaternionToYawRad(
    cmd->pose.pose.orientation.w,
    cmd->pose.pose.orientation.x,
    cmd->pose.pose.orientation.y,
    cmd->pose.pose.orientation.z);

  // put into cubic for x,y,theta as joystick left_x, left_y, right_x

  // find position/velocities
  std::vector<double> xpos0({current_x_, current_x_vel_});
  std::vector<double> xposf({cmd->pose.pose.position.x, cmd->twist.twist.linear.x});
  std::vector<double> ypos0({current_y_, current_y_vel_});
  std::vector<double> yposf({cmd->pose.pose.position.y, cmd->twist.twist.linear.y});
  std::vector<double> ang0({current_theta_rad_, current_theta_vel_rad_});
  std::vector<double> angf({current_theta_rad_ +
      ghost_util::SmallestAngleDistRad(theta_f, current_theta_rad_), cmd->twist.twist.angular.z});
  double pos_threshold = cmd->pose.pose.position.z;
  double theta_threshold = cmd->twist.twist.angular.x;

  RCLCPP_INFO(node_ptr_->get_logger(), "current x: %f, current x_vel: %f", xpos0[0], xpos0[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "current y: %f, current x_vel: %f", ypos0[0], ypos0[1]);
  RCLCPP_INFO(
    node_ptr_->get_logger(), "current theta: %f, current theta_vel: %f", ang0[0],
    ang0[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "final x: %f, final x_vel: %f", xposf[0], xposf[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "final y: %f, final x_vel: %f", yposf[0], yposf[1]);
  RCLCPP_INFO(node_ptr_->get_logger(), "final theta: %f, final theta_vel: %f", angf[0], angf[1]);

  // find final time
  double v_max = cmd->speed;
  double dist = Eigen::Vector2d(xposf[0] - xpos0[0], yposf[0] - ypos0[0]).norm();
  double t0 = 0;
  double tf = dist / v_max;
  int n = tf * 100;
  auto xpos_traj = computeCubicTraj(xpos0, xposf, t0, tf, n);
  auto ypos_traj = computeCubicTraj(ypos0, yposf, t0, tf, n);
  auto ang_traj = computeCubicTraj(ang0, angf, t0, tf, n);

  ghost_msgs::msg::RobotTrajectory trajectory_msg;
  ghost_msgs::msg::Trajectory x_t;
  ghost_msgs::msg::Trajectory y_t;
  ghost_msgs::msg::Trajectory theta_t;
  x_t.time = std::get<0>(xpos_traj);
  x_t.position = std::get<1>(xpos_traj);
  x_t.velocity = std::get<2>(xpos_traj);
  y_t.time = std::get<0>(ypos_traj);
  y_t.position = std::get<1>(ypos_traj);
  y_t.velocity = std::get<2>(ypos_traj);
  theta_t.time = std::get<0>(ang_traj);
  theta_t.position = std::get<1>(ang_traj);
  theta_t.velocity = std::get<2>(ang_traj);

  x_t.threshold = pos_threshold;
  y_t.threshold = pos_threshold;
  theta_t.threshold = theta_threshold;

  trajectory_msg.x_trajectory = x_t;
  trajectory_msg.y_trajectory = y_t;
  trajectory_msg.theta_trajectory = theta_t;

  RCLCPP_INFO(node_ptr_->get_logger(), "Generated Swerve Motion Plan");
  trajectory_pub_->publish(trajectory_msg);
}

MatrixXf CubicMotionPlanner::computeCubicCoeff(
  double t0, double tf, std::vector<double> vec_q0,
  std::vector<double> vec_qf)
{
  // vec_q0 = {position, velocity}
  // Ax = B
  // A = cubic function matrix
  // x = coefficients
  // B = initial and final values

  MatrixXf A(4, 4);
  MatrixXf B(4, 1);

  A(0, 0) = 1;
  A(0, 1) = t0;
  A(0, 2) = std::pow(t0, 2);
  A(0, 3) = std::pow(t0, 3);

  A(1, 0) = 0;
  A(1, 1) = 1;
  A(1, 2) = 2 * t0;
  A(1, 3) = 3 * std::pow(t0, 2);

  A(2, 0) = 1;
  A(2, 1) = tf;
  A(2, 2) = std::pow(tf, 2);
  A(2, 3) = std::pow(tf, 3);

  A(3, 0) = 0;
  A(3, 1) = 1;
  A(3, 2) = 2 * tf;
  A(3, 3) = 3 * std::pow(tf, 2);

  B(0, 0) = vec_q0[0];
  B(1, 0) = vec_q0[1];
  B(2, 0) = vec_qf[0];
  B(3, 0) = vec_qf[1];

  return A.inverse() * B;
}

std::tuple<std::vector<double>, std::vector<double>,
  std::vector<double>> CubicMotionPlanner::computeCubicTraj(
  std::vector<double> vec_q0,
  std::vector<double> vec_qf, double t0, double tf, int n)
{
  // A = coefficients
  // n = number of timesteps
  auto A = computeCubicCoeff(t0, tf, vec_q0, vec_qf);

  std::vector<double> a = {A(0, 0), A(1, 0), A(2, 0), A(3, 0)};

  std::vector<double> qd;
  std::vector<double> d_qd;
  std::vector<double> dd_qd;
  std::vector<double> time;
  double step = (tf - t0) / n;
  for (double t = t0; t < tf; t += step) {
    double qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3);
    double d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2);

    qd.push_back(qdi);
    d_qd.push_back(d_qdi);
    time.push_back(t);
  }

  return std::make_tuple(time, qd, d_qd);
}

} // namespace ghost_swerve

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto cubic_motion_planner = std::make_shared<ghost_swerve::CubicMotionPlanner>();
  cubic_motion_planner->configure("cubic_motion_planner");
  auto cubic_motion_planner_node = cubic_motion_planner->getROSNodePtr();
  rclcpp::spin(cubic_motion_planner_node);
  rclcpp::shutdown();
  return 0;
}
