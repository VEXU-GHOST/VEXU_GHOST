#include <ghost_tank/tank_odom.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>
#include <ghost_util/eigen_util.hpp>

namespace ghost_tank
{

TankOdometry::TankOdometry(int ticks_per_rotation, double wheel_radius_m, double wheelbase_m)
{
  double wheel_circumference = 2 * M_PI * wheel_radius_m;
  assert(wheel_circumference > 0);
  assert(ticks_per_rotation > 0);
  assert(wheelbase_m > 0);

  m_meters_per_tick = wheel_circumference / ticks_per_rotation;

  m_wheelbase = wheelbase_m;
  m_cur_pos = {0, 0, 0};
}
Eigen::Vector3d TankOdometry::update(
  long diff_l_wheel_pos,
  long diff_r_wheel_pos)
{
  double dl = diff_l_wheel_pos * m_meters_per_tick;
  double dr = diff_r_wheel_pos * m_meters_per_tick;

  // now we have absolute distances travelled by each wheel
  double dtheta = (dr - dl) / m_wheelbase;
  // we know the angle of the chassis

  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  rotation_matrix.block<2, 2>(0, 0) = Eigen::Rotation2Dd(m_cur_pos.z()).toRotationMatrix();

  // movement in the local frame
  Eigen::Vector3d local = Eigen::Vector3d::Zero();

  if (dr == dl) {
    local.x() = (dl + dr) / 2;
  } else {
    double dtheta = (dr - dl) / m_wheelbase;

    double rw = (dl / dtheta + dr / dtheta) / 2;
    local.x() = (rw) * sin(dtheta);
    local.y() = (rw) * (1 - cos(dtheta));
    local.z() = dtheta;
  }

  // rotate this movement into global frame
  m_cur_pos += rotation_matrix * local;

  m_cur_pos[2] = ghost_util::WrapAngle2PI(m_cur_pos[2]);
  return m_cur_pos;
}


Eigen::Vector3d TankOdometry::update(
  const Eigen::VectorX<long> & l_wheel_pos_arr,
  const Eigen::VectorX<long> & r_wheel_pos_arr)
{
  Eigen::VectorX<long> l_vel_arr =
    m_prev_l_ticks.size() ==
    l_wheel_pos_arr.size() ? (l_wheel_pos_arr - m_prev_l_ticks) : l_wheel_pos_arr;
  long l_wheel_pos = ghost_util::median(l_vel_arr);

  Eigen::VectorX<long> r_vel_arr =
    m_prev_r_ticks.size() ==
    r_wheel_pos_arr.size() ? (r_wheel_pos_arr - m_prev_r_ticks) : r_wheel_pos_arr;
  long r_wheel_pos = ghost_util::median(r_vel_arr);

  m_prev_l_ticks = l_wheel_pos_arr;
  m_prev_r_ticks = r_wheel_pos_arr;

  m_cur_pos = update(l_wheel_pos, r_wheel_pos);

  return m_cur_pos;
}

} // namespace ghost_tank
