#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "casadi/casadi.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/diff_drive_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ghost_util/unit_conversion_utils.hpp"

/*
Matrix dimensions are static! A is 5x5
*/
namespace diff_drive
{
class DiffDriveSS
{
public:
  DiffDriveSS();
  ~DiffDriveSS();
  void getA();
  void VltgToVelocity(std::vector<ghost_msgs::msg::V5MotorCommand, std::allocator<ghost_msgs::msg::V5MotorCommand>> & mtr_cmds);

  void DesiredTrajectoryCallback();
  // Approximate dynamics about state x_bar and control u_bar
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> ApproxDynamics(const Eigen::MatrixXd x_bar, const Eigen::MatrixXd u_bar, float del_T);
  // Nonlinear dynamics
  Eigen::MatrixXd F(const Eigen::MatrixXd x_bar, const Eigen::MatrixXd u_bar);
  // Propogate nonlinear dynamics over timestep del_T
  Eigen::VectorXd next_step(const Eigen::MatrixXd x_bar, const Eigen::MatrixXd u_bar, const float del_T);

  int GetNDim();
  int GetMDim();

private:
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  std::vector<Eigen::MatrixXd> Qk_data;
  std::vector<Eigen::MatrixXd> Rk_data;

  std::vector<Eigen::VectorXd> x_bar_data;
  std::vector<Eigen::Vector2d> u_bar_data;

  // Number of states
  int n_dim_;
  // Number of control variables
  int m_dim_;

  //// Physical Constants ////
  // Length between wheels
  float L_;
  // Radius of base to wheel
  float r_b_;
  // Wheel radius
  float r_w_;
  // TODO: UPDATE
  // Stall torque Nm
  float tau_stall_;
  // Volts
  float max_volt_;
  // RPM
  float free_speed_;
  // kg
  float m_;
  // Inertia kg m^2
  float I_;
  // Mass matrix
  Eigen::DiagonalMatrix<double, 3> M_;

};
}
