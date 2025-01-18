#include <ghost_tank/tank_odom.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <ghost_util/angle_util.hpp>
#include <ghost_util/unit_conversion_utils.hpp>

namespace ghost_tank
{

TankOdometry::TankOdometry(int ticks_per_rotation, double wheel_radius_m, double wheelbase_m)
{
  double wheel_circumference = 2 * M_PI * wheel_radius_m;
  m_meters_per_tick = wheel_circumference / ticks_per_rotation;

  m_wheelbase = wheelbase_m;
  m_cur_pos = {0, 0, 0};
}

template<typename T>
std::vector<T> subtractVectors(const std::vector<T> & vec1, const std::vector<T> & vec2)
{
  // Check if the vectors are of the same size
  if (vec1.size() != vec2.size()) {
    throw std::invalid_argument("Vectors must be of the same size for subtraction.");
  }

  std::vector<T> result(vec1.size());

  // Perform element-wise subtraction
  std::transform(vec1.begin(), vec1.end(), vec2.begin(), result.begin(), std::minus<T>());

  return result;
}

template<typename T>
T median(std::vector<T> & vec)
{
  if (vec.empty()) {
    throw std::invalid_argument("Cannot compute median of an empty vector.");
  }

  size_t size = vec.size();
  // Define the middle index
  auto middle = size / 2;

  if (size % 2 == 0) {
    // Even number of elements: find the two middle elements
    std::nth_element(vec.begin(), vec.begin() + middle, vec.end());
    T mid1 = vec[middle];   // The first middle element

    std::nth_element(vec.begin(), vec.begin() + middle - 1, vec.end());
    T mid2 = vec[middle - 1];   // The second middle element

    return (mid1 + mid2) / 2.0;   // Return the average of the two middle elements
  } else {
    // Odd number of elements: find the middle element
    std::nth_element(vec.begin(), vec.begin() + middle, vec.end());
    return vec[middle];   // Return the middle element
  }
}

Eigen::Vector3d TankOdometry::update(
  std::vector<long> l_wheel_pos,
  std::vector<long> r_wheel_pos)
{
    std::vector<long> l_vel_arr =
      !m_prev_l_ticks.empty() ? subtractVectors(l_wheel_pos, m_prev_l_ticks) : l_wheel_pos;

    double dl = median(l_vel_arr) * m_meters_per_tick;

    std::vector<long> r_vel_arr =
      !m_prev_r_ticks.empty() ? subtractVectors(r_wheel_pos, m_prev_r_ticks) : r_wheel_pos;
    double dr = median(r_vel_arr) * m_meters_per_tick;

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

    m_prev_l_ticks = l_wheel_pos;
    m_prev_r_ticks = r_wheel_pos;

    return m_cur_pos;
}

} // namespace ghost_tank
