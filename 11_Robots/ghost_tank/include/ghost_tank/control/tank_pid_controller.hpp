#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>

#include <ghost_util/angle_util.hpp>
#include <ghost_util/math_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>

#include <ghost_control/pid_controller.hpp>

#include <memory>

namespace ghost_tank
{

struct TankState
{
  // Constructor
  TankState(double p, double v, double a, double w)
  : position(p), velocity(v), angle(a), angular_velocity(w) {}

  // Subtraction operator
  TankState operator-(const TankState & other) const
  {
    return TankState{
      position - other.position,
      velocity - other.velocity,
      ghost_util::SmallestAngleDistRad(angle, other.angle),
      angular_velocity - other.angular_velocity
    };
  }

  double position{0.0};
  double velocity{0.0};
  double angle{0.0};
  double angular_velocity{0.0};
};

class TankPIDController
{
public:
  TankPIDController(const ghost_control::PIDConfig & linear_gains, const ghost_control::PIDConfig & angular_gains, float dt = 0.01)
  {
    linear_controller_ptr_ = std::make_shared<ghost_control::PIDController>(linear_gains, dt);
    angular_controller_ptr_ = std::make_shared<ghost_control::PIDController>(angular_gains, dt);
  }

  void reset()
  {
    linear_controller_ptr_->reset();
    angular_controller_ptr_->reset();
  }

  Eigen::Vector2d calculateDriveCommand(TankState current_state, TankState desired_state, bool backwards, bool ignore_lateral_error = false)
  {
    if (backwards) {
      desired_state.angle = ghost_util::FlipAnglePI(desired_state.angle);
    }

    // Calculate error
    auto error = desired_state - current_state;

    if (ignore_lateral_error) {
      error.position *= cos(error.angle);
      error.velocity *= cos(error.angle);
    }

    double output_linear = linear_controller_ptr_->calculateCommand(error.position, error.velocity);
    double output_angular = angular_controller_ptr_->calculateCommand(error.angle, error.angular_velocity);

    output_linear = ghost_util::clamp(output_linear, -1.0, 1.0);
    output_angular = ghost_util::clamp(output_angular, -1.0, 1.0);

    // Account for reverse mode
    output_linear *= (backwards) ? -1.0 : 1.0;

    return Eigen::Vector2d(output_linear, output_angular);
  }

private:
  std::shared_ptr<ghost_control::PIDController> linear_controller_ptr_;
  std::shared_ptr<ghost_control::PIDController> angular_controller_ptr_;
};

} // namespace ghost_tank
