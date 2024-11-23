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

// https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

    std::vector<long> l_vel_arr =
      !prev_l_ticks.empty() ? subtractVectors(l_wheel_pos, prev_l_ticks) : l_wheel_pos;

    double dl = median(l_vel_arr) * meters_per_tick;

    std::vector<long> r_vel_arr =
      !prev_r_ticks.empty() ? subtractVectors(r_wheel_pos, prev_r_ticks) : r_wheel_pos;
    double dr = median(r_vel_arr) * meters_per_tick;

    #define x cur_pos[0]
    #define y cur_pos[1]
    #define theta cur_pos[2]

    double dtheta = (dr - dl) / wheelbase;
    //dtheta *= 2;

    std::cout << "dl: " << dl << " dr: " << dr << " dtheta: " << dtheta * ghost_util::RAD_TO_DEG<< std::endl;
    //dtheta = w; // TODO garbage?? can we not use our actual angle??????

       Eigen::Matrix3d tfmat = Eigen::Matrix3d::Identity();
      tfmat.block<2,2>(0,0) = Eigen::Rotation2D<double> (dtheta).toRotationMatrix();



      Eigen::Matrix3d tftheta = Eigen::Matrix3d::Identity();
      tftheta.block<2,2>(0,0) = Eigen::Rotation2D<double> (theta).toRotationMatrix();

    if (dl == dr) {
      std::cerr << theta << " " << tftheta << " " << tftheta *
        Eigen::Vector3d(dr, 0, dtheta) << std::endl;
      cur_pos += tftheta *
        Eigen::Vector3d(dr, 0, dtheta);
    } else {
      double R;
      if (dtheta == 0) {
R = dr;
      }

       else
      R = (dl + dr) / 2 /  dtheta ;
      Eigen::Vector2d icc(x - R * sin(theta), y + R * cos(theta));
      #define iccx icc[0]
      #define iccy icc[1]

      cur_pos = Eigen::Vector3d(iccx, iccy, dtheta) +
        tfmat *
        (cur_pos - Eigen::Vector3d(iccx, iccy, 0));
    }

    cur_pos[2] = ghost_util::WrapAngle2PI(cur_pos[2]);

    prev_l_ticks = l_wheel_pos, prev_r_ticks = r_wheel_pos,
    // TODO: can we set this to our physical imu angle
    prev_angle = theta;
    return cur_pos;
  }
}

}//namespace ghost_tank
