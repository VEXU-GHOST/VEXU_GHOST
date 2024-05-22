#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "ghost_util/math_util.hpp"

namespace ghost_planners
{

class RobotTrajectory
{
public:
  struct Trajectory
  {
    Trajectory()
    {
      time_vector = std::vector<double>();
      position_vector = std::vector<double>();
      velocity_vector = std::vector<double>();

      threshold = 1.0;
    }
    double getPosition(double time) const
    {
      return ghost_util::clampedLinearInterpolate(time_vector, position_vector, time);
    }
    double getVelocity(double time) const
    {
      if (!time_vector.empty() && (time > *time_vector.end())) {
        return 0.5 * *velocity_vector.end();
      }
      return ghost_util::clampedLinearInterpolate(time_vector, velocity_vector, time);
    }
    bool checkPosition() const
    {
      return !position_vector.empty();
    }
    bool checkVelocity() const
    {
      return !velocity_vector.empty();
    }

    // values
    std::vector<double> time_vector;
    std::vector<double> position_vector;
    std::vector<double> velocity_vector;
    double threshold;

    bool operator==(const RobotTrajectory::Trajectory & rhs) const
    {
      return (time_vector == rhs.time_vector) && (position_vector == rhs.position_vector) &&
             (velocity_vector == rhs.velocity_vector);
    }
  };

  RobotTrajectory();
  Trajectory x_trajectory;
  Trajectory y_trajectory;
  Trajectory theta_trajectory;

  bool operator==(const RobotTrajectory & rhs) const
  {
    return (x_trajectory == rhs.x_trajectory) &&
           (y_trajectory == rhs.y_trajectory) &&
           (theta_trajectory == rhs.theta_trajectory);
  }

  bool isNotEmpty() const
  {
    return !x_trajectory.time_vector.empty() &&
           !y_trajectory.time_vector.empty() &&
           !theta_trajectory.time_vector.empty();
  }
};

} // namespace ghost_planners
