#include "ilqr.hpp"

// TODO: put exceptions around push backs so it doesn't segfault

namespace ilqr
{

Ilqr::Ilqr() {
  // Load YAML
  YAML::Node config_ = YAML::LoadFile("ilqr_config.yaml");
  diff_drive_ptr_ = std::make_unique<diff_drive::DiffDriveSS>();
  x0_data_.reserve(horizon_ / del_T_);
  u0_data_.reserve((horizon_ - del_T_) / del_T_);
  del_u_star_data_.reserve((horizon_ - del_T_) / del_T_);
  A_data_.reserve(horizon_ / del_T_);
  B_data_.reserve(horizon_ / del_T_);

  n_dim_ = config_["n_dim"].as<int>();
  m_dim_ = config_["m_dim"].as<int>();
  x0_ = Eigen::Map<Eigen::Vector<double, 5>>(config_["x0"].as<std::vector<double>>().data());
  u0_ = Eigen::Map<Eigen::Vector<double, 2>>(config_["u0"].as<std::vector<double>>().data());
};

Ilqr::~Ilqr(){};

/*
Updates state when map position is updated from localization
*/ 
void Ilqr::UpdateX0(Eigen::MatrixXd new_state){
  x0_ = new_state;
}

/*
Updates to latest control voltage
*/
void Ilqr::UpdateU0(Eigen::MatrixXd new_control){
  u0_ = new_control;
}

void Ilqr::InitialRollout(){
  Eigen::VectorXd x_n; 
  Eigen::Vector2d u_n; 

  for(int i = 0; i < horizon_ / del_T_; i++){
    if(i == 0){
      x0_data_.push_back(x0_);
      u0_data_.push_back(u0_);
    }
    else{
      x_n = diff_drive_ptr_->next_step(x0_data_.back(), u0_data_.back(), del_T_);
      u_n = Eigen::Vector2d::Zero(diff_drive_ptr_->GetMDim());
      x0_data_.push_back(x_n);
      u0_data_.push_back(u_n);
    }
  }

// Linearize initial trajectories at every time step
  for(int i = 0; i < horizon_ / del_T_; i++){
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> AkBk = diff_drive_ptr_->ApproxDynamics(x0_data_[i], u0_data_[i], del_T_);
    A_data_.push_back(std::get<0>(AkBk));
    B_data_.push_back(std::get<1>(AkBk));
  }
}

void Ilqr::BackwardPass(std::vector<Eigen::VectorXd> x_bar_data, std::vector<Eigen::Vector2d> u_bar_data){
  int data_i = horizon_ / del_T_;
  bool at_term_cost = true;
  Eigen::MatrixXd sn;
  Eigen::MatrixXd Sn;
  // auto del_x = casadi::SX::zeros(n_dim_);
  Eigen::VectorXd del_x;

  auto x = casadi::SX::sym("x");
  auto y = casadi::SX::sym("y");
  auto tht = casadi::SX::sym("tht");
  auto vl = casadi::SX::sym("vl");
  auto vr = casadi::SX::sym("vr");

  // del_x << x, y, tht, vl, vr;

  for(int n = horizon_; n > 0; n-= del_T_){
    if (at_term_cost) {
      std::vector<Eigen::MatrixXd> vn_gains = ApproxVN(x_bar_data[data_i], u_bar_data[data_i]);
      sn = vn_gains[0];
      Sn = vn_gains[1];
      Eigen::VectorXd xN = x_bar_data.back();

      V_data_.push_back(0.5 * xN.transpose() * Sn * xN + sn * xN);
      at_term_cost = false;
    }

    // If we are not at timestep 0
    if (data_i != 0){
      std::vector<Eigen::MatrixXd> c_gains = ApproxC(x_bar_data[data_i], u_bar_data[data_i]);
      Eigen::MatrixXd lx = c_gains[0];
      Eigen::MatrixXd lu = c_gains[1];
      Eigen::MatrixXd lxx = c_gains[2];
      Eigen::MatrixXd luu = c_gains[3];
      Eigen::MatrixXd Ak = A_data_[data_i];
      Eigen::MatrixXd Bk = B_data_[data_i];

      Eigen::MatrixXd Qx = lx + Ak * sn.transpose();
      Eigen::MatrixXd Qu = lu + Bk * sn;
      Eigen::MatrixXd Qxx = lxx + Ak.transpose() * Sn * Ak;
      Eigen::MatrixXd Quu = luu + Bk.transpose() * Sn * Ak;
      Eigen::MatrixXd Qux = Bk.transpose() * Sn * Ak;

      // Solve for Value Function at n + 1
      Eigen::VectorXd d = -1 * Quu.inverse() * Qu;
      Eigen::MatrixXd Kk = -1 * Quu.inverse() * Qux;
      Eigen::VectorXd del_u_star = d + (Kk * del_x);
      Eigen::MatrixXd del_V = 0.5 * d.transpose() * Quu * d + d.transpose() * Qu;
      
      V_data_.insert(V_data_.begin(), V_data_.front() + del_V);
      del_u_star_data_.insert(del_u_star_data_.begin(), del_u_star);

      sn = (Qx + Kk.transpose() * Quu * d + Kk.transpose() * Qu + Qux.transpose() * d).transpose();
      Sn = (Qxx + Kk.transpose() * Quu * Kk + Kk.transpose() * Qux + Qux.transpose() * Kk).transpose();
    }
  }
  
}
void Ilqr::ForwardPass(std::vector<Eigen::VectorXd> x_bar_data, std::vector<Eigen::Vector2d> del_u_star_data, std::vector<Eigen::Vector2d> old_u_data){
}
/*
Quad Approximate cost equation at xk uk
*/
std::vector<Eigen::MatrixXd> Ilqr::ApproxC(Eigen::VectorXd x_k, Eigen::Vector2d u_k){
  Eigen::MatrixXd lx = Q_ * x_k;
  Eigen::MatrixXd lu = Q_ * u_k;
  Eigen::MatrixXd lxx = Q_;
  Eigen::MatrixXd luu = R_;
  // lxu and lux are zero for cost function without bilinear term: xT * Q * x +  uT * R * u
  
  std::vector<Eigen::MatrixXd> c_gains = {lx, lu, lxx, luu};
  return c_gains;
}
/*
Approximate quadratic cost at terminal state
*/
std::vector<Eigen::MatrixXd> Ilqr::ApproxVN(Eigen::VectorXd x_N, Eigen::Vector2d u_N){
  // Terminal state cost V_N = (x - x_bar)^T * Q * (x - x_bar)
  // dVN_dx == 0 given equation below, substituting x and x_syms with x_N
  // [-q1_*(2*x - 2*x_syms), -q2_*(2*y - 2*y_syms), -q3_*(2*tht - 2*tht_syms), -q4_*(2*vl - 2*vl_syms), -q5_*(2*vr - 2*vr_syms)]
  Eigen::VectorXd dV_N = Eigen::VectorXd::Zero(n_dim_);
  Eigen::MatrixXd ddV_N = Q_;

  std::vector<Eigen::MatrixXd> vn_gains = {dV_N, ddV_N};

  return vn_gains;
}

}
//namespace ilqr