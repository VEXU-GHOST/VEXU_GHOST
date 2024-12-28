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
Eigen::MatrixXd DiffDriveSS::F(const Eigen::MatrixXd x_bar, const Eigen::MatrixXd u_bar)
{
  // Unpack state theta
  float tht = x_bar(2);

  // wheel states to base states Jacobian, J
  Eigen::MatrixXd J;
  J << cos(tht), cos(tht),
       sin(tht), sin(tht),
        -1 / L_,   1 / L_;

  // base states to wheel states Jacobian, J_plus
  Eigen::MatrixXd J_plus(m_dim_, 3);

  J_plus << cos(tht), sin(tht), -L_,
            cos(tht), sin(tht),  L_;

  Eigen::MatrixXd dJ_plus(m_dim_, 3);
  dJ_plus << -1 * sin(tht), cos(tht), 0,
             -1 * sin(tht), cos(tht), 0;

  Eigen::MatrixXd up_right = J;
  Eigen::MatrixXd low_right = dJ_plus * J - J_plus * M_.inverse() * J_plus.transpose() * tau_stall_ / (pow(r_w_, 2) * free_speed_);

  // Padding
  Eigen::MatrixXd up_left =
    Eigen::MatrixXd::Zero(n_dim_ - low_right.rows(), n_dim_ - up_right.cols());
  Eigen::MatrixXd low_left =
    Eigen::MatrixXd::Zero(n_dim_ - up_right.rows(), n_dim_ - low_right.cols());

  Eigen::MatrixXd H;
  H <<  up_left,  up_right,
       low_left, low_right;

  Eigen::MatrixXd C;
  C << low_left.transpose(),
       J_plus * M_.inverse() * J_plus.transpose() * tau_stall_ / (pow(r_w_, 2) * free_speed_);

  return H * x_bar + C * u_bar;
}


/* 
  Approximate dynamics about kth state x_bar and control u_bar over timestep del_T
  Returns tuple (A_k, B_k)
*/
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> DiffDriveSS::ApproxDynamics(const Eigen::MatrixXd x_bar, const Eigen::MatrixXd u_bar, const float del_T)
{
  double tht = x_bar(2);
  double vl = x_bar(3);
  double vr = x_bar(4);

  Eigen::MatrixXd A_J33 = -1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() + pow(L_, 2) * M_.inverse()));
  Eigen::MatrixXd A_J34 = -1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() - pow(L_, 2) * M_.inverse()));
  Eigen::MatrixXd A_J43 = -1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() - pow(L_, 2) * M_.inverse()));
  Eigen::MatrixXd A_J44 = -1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() + pow(L_, 2) * M_.inverse()));
  
  // dF_dx
  Eigen::MatrixXd A_J(n_dim_, n_dim_);
  A_J <<
    0, 0, -vl * sin(tht) - vr * sin(tht), cos(tht), cos(tht),
    0, 0,  vl * cos(tht) + vr * cos(tht), sin(tht), sin(tht),
    0, 0,                              0,  -1 / L_,   1 / L_,
    0, 0,                              0,    A_J33,    A_J34,
    0, 0,                              0,    A_J43,    A_J44;

  Eigen::MatrixXd B_J30 = 1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() + pow(L_, 2) * M_.inverse()));
  Eigen::MatrixXd B_J31 = 1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() - pow(L_, 2) * M_.inverse()));
  Eigen::MatrixXd B_J40 = 1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() - pow(L_, 2) * M_.inverse()));
  Eigen::MatrixXd B_J41 = 1 / (free_speed_ * pow(r_w_, 2)) * (tau_stall_ * (pow(cos(tht), 2) * M_.inverse() + pow(sin(tht), 2) * M_.inverse() + pow(L_, 2) * M_.inverse()));

  // dF_du
  Eigen::MatrixXd B_J(m_dim_, m_dim_);
  B_J <<
        0,     0,
        0,     0,
        0,     0,
    B_J30, B_J31,
    B_J40, B_J41;

  Eigen::MatrixXd A_k = Eigen::MatrixXd::Identity(n_dim_, n_dim_) + del_T * A_J;
  Eigen::MatrixXd B_k = del_T * B_J;

  return std::tuple(A_k, B_k);
}

/*
  For a given control u_bar, returns next state x_bar
*/
Eigen::VectorXd DiffDriveSS::next_step(const Eigen::MatrixXd x_bar, const Eigen::MatrixXd u_bar, const float del_T){
  Eigen::MatrixXd Fk = F(x_bar, u_bar) * del_T;
  Eigen::VectorXd x_next = x_bar + Fk;
  return x_next;
}

/*
  Getters
*/
int DiffDriveSS::GetNDim(){
  return n_dim_;  
}
int DiffDriveSS::GetMDim(){
  return m_dim_;
}

}
// namespace diff_drive
