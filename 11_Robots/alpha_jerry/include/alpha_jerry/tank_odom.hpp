#pragma once
#include <vector>
#include <eigen3/Eigen/Geometry>


namespace alpha_jerry
{

class TankOdometry
{
public:
  // Constructor


  /// @brief Initialize Tank Odometry
  /// @param ticks_per_wheel_rotation How many rotations of the physical wheel translate into motor encoder ticks?
  ///                                  Include motor specs & any gearing after
  /// @param wheel_radius_m Wheel radius in meters
  /// @param wheelbase_m Distance between the centers of the left and right wheel tracks in meters
  TankOdometry(int ticks_per_wheel_rotation, double wheel_radius_m, double wheelbase_m);

  Eigen::Vector3d getPose() const
  {
    return m_cur_pos;
  }

  /// @brief Update the odometry according to new encoder state.
  /// @param l_wheel_pos Left wheel. A variable-length vector of absolute motor encoder values in units specified in constructed.
  ///                     The median difference between this and the value from the last call will be used to increment odometry.
  /// @param r_wheel_pos Same as above but right wheel.
  /// @return Vector3d new, updated pose.
  Eigen::Vector3d update(
    const Eigen::VectorX<long> & l_wheel_pos,
    const Eigen::VectorX<long> & r_wheel_pos);

  /// @brief Update the odometry according to wheel rotation difference from the last time step.
  /// @param diff_l_wheel_pos Left wheel. Change in motor rotation ticks since last call to this function.
  /// @param diff_r_wheel_pos Right wheel. Same as above.
  /// @return Vector3d new, updated pose.
  Eigen::Vector3d update(long diff_l_wheel_pos, long diff_r_wheel_pos);


  void resetPose()
  {
    setPose({0, 0, 0});
  }

  void setPose(const Eigen::Vector3d & p)
  {
    m_cur_pos = p;
  }

private:
  Eigen::VectorX<long> m_prev_l_ticks;
  Eigen::VectorX<long> m_prev_r_ticks;

  Eigen::Vector3d m_cur_pos = Eigen::Vector3d::Zero();

  double m_meters_per_tick;
  double m_wheelbase;
};
} //namespace alpha_jerry
