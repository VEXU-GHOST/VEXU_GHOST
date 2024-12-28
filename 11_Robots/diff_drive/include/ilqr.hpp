#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "diff_drive_ss.hpp"
#include "yaml-cpp/yaml.h"
#include <casadi/casadi.hpp>

#include <vector>

/*
Iterative Linear Quadratic Regulator for Differential Drive Robot
Input: Desired Trajectory, time horizon
Output: Optimal state trajectory given cost and current state
*/
namespace ilqr
{
class Ilqr
{
public:
  Ilqr();
  ~Ilqr();

  void InitialRollout();
  void BackwardPass(std::vector<Eigen::VectorXd> x_bar_data, std::vector<Eigen::Vector2d> u_bar_data);
  void ForwardPass(std::vector<Eigen::VectorXd> x_bar_data, std::vector<Eigen::Vector2d> del_u_star_data, std::vector<Eigen::Vector2d> old_u_data);
  std::vector<Eigen::MatrixXd> ApproxC(Eigen::VectorXd x_k, Eigen::Vector2d u_k);
  std::vector<Eigen::MatrixXd> ApproxVN(Eigen::VectorXd x_N, Eigen::Vector2d u_N);

  void UpdateX0(const Eigen::MatrixXd new_state);
  void UpdateU0(const Eigen::MatrixXd new_control);

private:
  int n_dim_;
  int m_dim_;
  float horizon_;
  float del_T_;
  std::unique_ptr<diff_drive::DiffDriveSS> diff_drive_ptr_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  Eigen::MatrixXd x0_;
  Eigen::MatrixXd u0_;

  std::vector<Eigen::MatrixXd> x0_data_;
  std::vector<Eigen::MatrixXd> u0_data_;

  std::vector<Eigen::MatrixXd> A_data_;
  std::vector<Eigen::MatrixXd> B_data_;
  std::vector<Eigen::MatrixXd> V_data_;

  std::vector<Eigen::MatrixXd> del_u_star_data_;


};

}
