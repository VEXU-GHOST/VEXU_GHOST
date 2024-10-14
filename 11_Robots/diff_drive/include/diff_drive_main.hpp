#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/diff_drive_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ghost_util/angle_util.hpp"
#include "diff_drive_ss.hpp"
/*
Matrix dimensions are static! A is 5x5
*/


namespace diff_drive
{

class DiffDriveNode : public rclcpp::Node
{
public:
  DiffDriveNode();
  std::unique_ptr<diff_drive::DiffDriveSS> ss;

  // Subscribers
  rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr mtr_vltg_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<ghost_msgs::msg::DiffDriveState>::SharedPtr diff_drive_ss_pub_;

  void MtrVltgCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void getA();

private:
  unsigned long int n_dim_;
  unsigned long int m_dim_;

  // Motor Properties
  float tau_stall_;
  float v_max_;

  // Vehicle Geometry
  float r_wheel_;
  float r_base_;


  // Initialize. Replace with EKF data when running MPC and hardware
  Eigen::VectorXf init_state_;

  // State Space Matrices
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::VectorXd state_;
};

}
// namespace diff_drive
