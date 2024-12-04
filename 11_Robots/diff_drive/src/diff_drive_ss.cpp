#include "diff_drive_ss.hpp"
using namespace casadi;

namespace diff_drive
{
DiffDriveSS::DiffDriveSS()
{
// Number of states
  n_dim_ = 5;
// Number of control variables
  m_dim_ = 2;

  //// Physical Constants ////
  // Length between wheels
  L_ = 10 * ghost_util::INCHES_TO_METERS;
  // Radius of base to wheel
  r_b_ = L_ / 2;
  // Wheel radius
  r_w_ = 2.75 * 0.5 * ghost_util::INCHES_TO_METERS;

  // TODO: UPDATE
  // Stall torque Nm
  tau_stall_ = 2.42;
  // Volts
  max_volt_ = 10;
  // RPM
  free_speed_ = 4110;
  // kg
  m_ = 20 * ghost_util::LBS_TO_KG;
  // Inertia kg m^2
  I_ = 2;
  // Mass matrix
  M_.diagonal() << (m_, m_, I_);
}

DiffDriveSS::~DiffDriveSS() {}
void DiffDriveSS::DesiredTrajectoryCallback() {}


// Nonlinear dynamics
Eigen::MatrixXd DiffDriveSS::F(Eigen::MatrixXd x_bar, Eigen::MatrixXd u_bar)
{
  // Unpack state theta
  float tht = x_bar(2);

  // wheel states to base states Jacobian, J
  Eigen::MatrixXd J;
  J << cos(tht), cos(tht),
    sin(tht), sin(tht),
    -1 / L_, 1 / L_;

  // base states to wheel states Jacobian, J_plus
  Eigen::MatrixXd J_plus(m_dim_, 3);

  J_plus << cos(tht), sin(tht), -L_,
    cos(tht), sin(tht), L_;

  Eigen::MatrixXd dJ_plus(m_dim_, 3);
  dJ_plus << -1 * sin(tht), cos(tht), 0,
    -1 * sin(tht), cos(tht), 0;

  Eigen::MatrixXd up_right = J;
  Eigen::MatrixXd low_right = dJ_plus * J - J_plus * M_.inverse() * J_plus.transpose() *
    tau_stall_ /
    (pow(r_w_, 2) * free_speed_);

  // Padding
  Eigen::MatrixXd up_left =
    Eigen::MatrixXd::Zero(n_dim_ - low_right.rows(), n_dim_ - up_right.cols());
  Eigen::MatrixXd low_left =
    Eigen::MatrixXd::Zero(n_dim_ - up_right.rows(), n_dim_ - low_right.cols());

  Eigen::MatrixXd H;
  H << up_left, up_right,
    low_left, low_right;

  Eigen::MatrixXd C;
  C << low_left.transpose(),
    J_plus * M_.inverse() * J_plus.transpose() * tau_stall_ / (pow(r_w_, 2) * free_speed_);

  return H * x_bar + C * u_bar;
}


// Approximate dynamics about state x_bar and control u_bar
void DiffDriveSS::ApproxDynamics(Eigen::MatrixXd x_bar, Eigen::MatrixXd u_bar)
{

}

}
// namespace diff_drive
