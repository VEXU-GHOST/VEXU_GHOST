#pragma once
#include <cmath>
#include <memory>

#include <eigen3/Eigen/Core>

#include <ghost_control/pid_controller.hpp>

namespace ghost_tank
{

// Per-axis gains for the closed-loop velocity controller: PD on velocity error,
// a velocity feedforward term, and a static feedforward term.
struct VelocityAxisConfig
{
  double p = 0.0;          // proportional gain on (normalized) velocity error
  double d = 0.0;          // derivative gain on velocity error
  double ff = 0.0;         // feedforward gain on the commanded velocity (kV)
  double static_ff = 0.0;  // static feedforward magnitude (kS), added with the sign of the command to overcome static friction
};

// Closed-loop chassis velocity controller used by the velocity-tracking BT nodes
// (MoveVelocityPDFF, FollowPathControllerServerPDFF). Mirrors TankPIDController:
// wraps one ghost_control::PIDController per axis -- PD on velocity error, ki
// unused -- and adds a velocity feedforward term plus a static feedforward term.
//
// Per axis: cmd = kS * sign(cmd_frac) + kV * cmd_frac + p * (cmd_frac - meas_frac)
//                 + d * d/dt(err)
// where kV is the velocity feedforward gain (ff), kS is the static feedforward
// magnitude (static_ff), and *_frac is a velocity normalized to a fraction of the
// chassis max velocity. The static feedforward is added to the command (not
// clamped onto the output), so it does not create a relay/limit-cycle near the
// setpoint. Output is the (forward, angular) arcade command, pre-normalization.
class VelocityController
{
public:
  VelocityController(const VelocityAxisConfig & linear, const VelocityAxisConfig & angular)
  : linear_cfg_(linear), angular_cfg_(angular)
  {
    linear_controller_ptr_ = std::make_shared<ghost_control::PIDController>(
      ghost_control::PIDConfig{linear.p, 0.0, linear.d});
    angular_controller_ptr_ = std::make_shared<ghost_control::PIDController>(
      ghost_control::PIDConfig{angular.p, 0.0, angular.d});
  }

  // Clears the per-axis PID state and the D-term history. Call on each onStart().
  void reset()
  {
    linear_controller_ptr_->reset();
    angular_controller_ptr_->reset();
    have_prev_ = false;
    prev_lin_err_ = 0.0;
    prev_ang_err_ = 0.0;
  }

  // cmd_frac / meas_frac: commanded & measured velocity as a fraction of the
  // chassis max velocity, per axis. dt: seconds since the previous call (D term).
  Eigen::Vector2d calculateCommand(
    double lin_cmd_frac, double lin_meas_frac,
    double ang_cmd_frac, double ang_meas_frac, double dt)
  {
    double lin_err = lin_cmd_frac - lin_meas_frac;
    double ang_err = ang_cmd_frac - ang_meas_frac;

    // Derivative of error; zero on the first tick (no prior sample / dt yet).
    double lin_derr = 0.0;
    double ang_derr = 0.0;
    if (have_prev_ && dt > 1.0e-6) {
      lin_derr = (lin_err - prev_lin_err_) / dt;
      ang_derr = (ang_err - prev_ang_err_) / dt;
    }
    prev_lin_err_ = lin_err;
    prev_ang_err_ = ang_err;
    have_prev_ = true;

    // PD on velocity error + velocity feedforward (kV) + static feedforward (kS),
    // both passed as the additional term added to the PID output.
    double fwd = linear_controller_ptr_->calculateCommand(
      lin_err, lin_derr,
      linear_cfg_.ff * lin_cmd_frac + staticFF(lin_cmd_frac, linear_cfg_.static_ff));
    double ang = angular_controller_ptr_->calculateCommand(
      ang_err, ang_derr,
      angular_cfg_.ff * ang_cmd_frac + staticFF(ang_cmd_frac, angular_cfg_.static_ff));

    return Eigen::Vector2d(fwd, ang);
  }

private:
  // Static feedforward: kS in the direction of the command, zero when there is
  // effectively no command (so the drive can still brake to a stop). Unlike the
  // old output floor this is added to the command, not clamped onto the output,
  // so it does not create a relay/limit-cycle near the setpoint.
  static double staticFF(double cmd_frac, double k_s)
  {
    if (k_s <= 0.0 || std::fabs(cmd_frac) < 1.0e-4) { return 0.0; }
    return std::copysign(k_s, cmd_frac);
  }

  VelocityAxisConfig linear_cfg_;
  VelocityAxisConfig angular_cfg_;
  std::shared_ptr<ghost_control::PIDController> linear_controller_ptr_;
  std::shared_ptr<ghost_control::PIDController> angular_controller_ptr_;

  double prev_lin_err_{0.0};
  double prev_ang_err_{0.0};
  bool have_prev_{false};
};

} // namespace ghost_tank
