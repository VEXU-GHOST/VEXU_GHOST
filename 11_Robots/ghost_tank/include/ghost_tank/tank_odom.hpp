#pragma once
#include <vector>
#include <eigen3/Eigen/Geometry>


namespace ghost_tank
{

class TankOdometry
{
public:
  // Constructor
  TankOdometry(int ticks_per_wheel_rotation, double wheel_radius_m, double wheelbase_m);

  Eigen::Vector3d getPose() const
  {
    return m_cur_pos;
  }

  Eigen::Vector3d update(Eigen::VectorX<long>  l_wheel_pos, Eigen::VectorX<long>  r_wheel_pos);
  Eigen::Vector3d update( long  diff_l_wheel_pos, long  diff_r_wheel_pos);


  void resetPose()
  {
    setPose({0, 0, 0});
  }

  void setPose(Eigen::Vector3d p)
  {
    m_cur_pos = p;
  }

private:
  Eigen::VectorX<long> m_prev_l_ticks ;
  Eigen::VectorX<long> m_prev_r_ticks ;

  Eigen::Vector3d m_cur_pos;

  double m_meters_per_tick;
  double m_wheelbase;
};
} //namespace ghost_tank
