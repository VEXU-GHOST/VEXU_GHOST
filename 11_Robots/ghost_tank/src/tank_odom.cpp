#include "ghost_tank/tank_odom.hpp"
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
  meters_per_tick = wheel_circumference / ticks_per_rotation;
  //std::cout << "meters per tick" << meters_per_tick << std::endl;

  wheelbase = wheelbase_m;
  cur_pos = {0, 0, 0};
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
    T mid1 = vec[middle];      // The first middle element

    std::nth_element(vec.begin(), vec.begin() + middle - 1, vec.end());
    T mid2 = vec[middle - 1];      // The second middle element

    return (mid1 + mid2) / 2.0;      // Return the average of the two middle elements
  } else {
    // Odd number of elements: find the middle element
    std::nth_element(vec.begin(), vec.begin() + middle, vec.end());
    return vec[middle];      // Return the middle element
  }
}

Eigen::Vector3d TankOdometry::update(
  std::vector<long> l_wheel_pos,
  std::vector<long> r_wheel_pos)
{
  {

// NO // https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

    std::vector<long> l_vel_arr =
      !prev_l_ticks.empty() ? subtractVectors(l_wheel_pos, prev_l_ticks) : l_wheel_pos;

    double dl = median(l_vel_arr) * meters_per_tick;

    std::vector<long> r_vel_arr =
      !prev_r_ticks.empty() ? subtractVectors(r_wheel_pos, prev_r_ticks) : r_wheel_pos;
    double dr = median(r_vel_arr) * meters_per_tick;

    double dtheta = (dr - dl) / wheelbase;
    //dtheta *= 2;

    std::cout << "dl: " << dl << " dr: " << dr << " dtheta: " << dtheta * ghost_util::RAD_TO_DEG<< std::endl;
    //dtheta = w; // TODO garbage?? can we not use our actual angle??????

    Eigen::Vector2d local = {}; 

    std::cout << "dr/dtheta: " << dr/dtheta << " corelogic: " << dr / dtheta + wheelbase / 2 << std::endl;

  //https://github.com/OkapiLib/OkapiLib/blob/master/src/api/odometry/twoEncoderOdometry.cpp
  if (dtheta != 0) {
      local[0] = 2 * std::sin(dtheta / 2) * 0;//chassisScales.middleWheelDistance.convert(meter);
      local[1] = 2 * std::sin(dtheta / 2) *
                  (-dr / dtheta + wheelbase / 2);
  } else {
      local[0] = 0;
      local[1] = dr;
  }

  double avgA = cur_pos[2] + dtheta/2 ;
  std::cout << "localx: " << local[0] << " localy: " << local[1] << " avgA: " << avgA << std::endl;

  //double polarR = std::sqrt(localOffX * localOffX + localOffY * localOffY);
  double polarR = std::sqrt(local[0] * local[0] + local[1] * local[1]);
  //double polarR = local[1];
  double polarA = std::atan2(local[1], local[0]) - avgA;
  std::cout << "polarR: " << polarR << " polarA: " << polarA << std::endl;

  double dX = std::sin(polarA) * polarR;
  double dY = std::cos(polarA) * polarR;


  if (isnan(dX)) {
    dX = 0;
  }

  if (isnan(dY)) {
    dY = 0;
  }


  if (isnan(dtheta)) {
    dtheta = 0;
  }


cur_pos[0] += dX;
cur_pos[1] += dY;
cur_pos[2] += dtheta;

    cur_pos[2] = ghost_util::WrapAngle2PI(cur_pos[2]);

    prev_l_ticks = l_wheel_pos, prev_r_ticks = r_wheel_pos;

    printf("\rpos: x: %.2f y: %.2f theta: %.2f\n", getPose().x(), getPose().y(), getPose().z());
    // TODO: can we set this to our physical imu angle
    return cur_pos;
  }
}

}//namespace ghost_tank
