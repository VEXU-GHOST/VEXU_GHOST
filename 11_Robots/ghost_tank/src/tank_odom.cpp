#include "ghost_tank/tank_odom.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <ghost_util/angle_util.hpp>


namespace ghost_tank
{

TankOdometry::TankOdometry(int ticks_per_rotation, double wheel_radius_m, double wheelbase_m)
{
  double wheel_circumference = 2 * M_PI * wheel_radius_m;
  meters_per_tick = wheel_circumference / ticks_per_rotation;
  //std::cout << "meters per tick" << meters_per_tick << std::endl;

  wheelbase = wheelbase_m;
  cur_pos = {0,0,0};
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

Eigen::Vector3d TankOdometry::update(std::vector<long> l_wheel_pos,
  std::vector<long> r_wheel_pos, double angle_rad)
{
  {

// https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
// diff them

//std::cerr << 1.1;
    std::vector<long> l_vel_arr = !prev_l_ticks.empty() ? subtractVectors(l_wheel_pos, prev_l_ticks) : l_wheel_pos;

    double dl = median(l_vel_arr) * meters_per_tick;
 //   std::cout <<std::endl << "DL"<< dl<< "DL" << median(l_vel_arr) << std::endl;

//std::cerr << 1.2;
    std::vector<long> r_vel_arr = !prev_r_ticks.empty() ? subtractVectors(r_wheel_pos, prev_r_ticks) : r_wheel_pos;
    double dr = median(r_vel_arr) * meters_per_tick;

    double dtheta = angle_rad - prev_angle;

//std::cerr << 1.4;

#define x cur_pos[0]
#define y cur_pos[1]
#define theta cur_pos[2]


//theta;
//std::cerr << 1.41;
    //Eigen::Vector3d dd(median(l_vel_arr), median(r_vel_arr), );


//assert(wheelbase != 0);
//assert(dl !=0);
//assert(dr !=0);

// omega = dtheta/dt

double w = (dr - dl)/wheelbase;
dtheta = w; // TODO garbage?? can we not use our actual angle??????

    //double R = (2) * ((dl + dr) / (dl - dr)) ;

//iccy;
//std::cerr << 1.5;

Eigen::Matrix3d tfmat;
tfmat << cos(dtheta), -sin(dtheta), 0.,
 sin(dtheta), cos(dtheta), 0.,
  0., 0., 1.;

//std::cout << std::endl << tfmat << std::endl;

//std::cout << std::endl << "icc" << icc << std::endl;
//std::cerr << 1.6;
if (dl == dr) {
    cur_pos += tfmat *
      Eigen::Vector3d(dr, 0, dtheta);
} else { 
    double R = (dl + dr) / w / 2;
//std::cerr << 1.45;
    Eigen::Vector2d icc(x - R * sin(theta), y + R * cos(theta));
#define iccx icc[0]
#define iccy icc[1]

    cur_pos = Eigen::Vector3d(iccx, iccy, dtheta) +
      tfmat *
      (cur_pos - Eigen::Vector3d(iccx, iccy, 0));
}

  cur_pos[2] = ghost_util::WrapAngle2PI(cur_pos[2]);

//      cur_pos[2] = angle_rad; // TODO: how do i better integrate imu stuff into my stuff of stuff

//std::cerr << 1.7;

// convert ticks to meters
//
//    // calculate local x and y
//    float localX = 0;
//    float localY = 0;
//    if (deltaHeading == 0) { // prevent divide by 0
//        localX = deltaX;
//        localY = deltaY;
//    } else {
//        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading);// + horizontalOffset);
//        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading);// + verticalOffset);
//    }
//
//
//
//
//
//    auto rotate_base_to_odom = Eigen::Rotation2D<double>(m_odom_angle).toRotationMatrix();
//    m_odom_loc += rotate_base_to_odom *
//      Eigen::Vector2d(m_base_vel_curr.x(), m_base_vel_curr.y()) * 0.01;
//    m_odom_angle += m_base_vel_curr.z() * 0.01;
//    m_odom_angle = ghost_util::WrapAngle2PI(m_odom_angle);
//
    prev_l_ticks = l_wheel_pos, prev_r_ticks = r_wheel_pos,
    prev_angle = angle_rad;
    return cur_pos;
  }
}

//std::vector<double> TankOdometry::getRobotWorldPosition(double in_left_encoder, double in_right_encoder){

// double dl = (in_left_encoder - left_encoder)/ticks * circumference;
// double dr = (in_right_encoder - left_encoder)/ticks * circumference;
// double dcenter = (dl+dr)/2.0;
// theta += (dr - dl)/ (2.0*Rw);

//
// current_xpos = prev_xpos + dcenter*std::cos(theta);
// current_ypos = prev_ypos + dcenter*std::sin(theta);

// left_encoder = in_left_encoder;
// right_encoder = in_right_encoder;
//
// prev_xpos = current_xpos;
// prev_ypos = current_ypos;

// return {current_xpos, current_ypos, theta};
//}

//   void TankOdometry::resetEncoders(){
//   double prev_xpos = 0;
//   double prev_ypos = 0;
//   double current_xpos = 0;
//   double current_ypos = 0;
//   double left_encoder = 0;
//   double right_encoder = 0;
//   double theta = 0;
//    }


//std::Vector

}//namespace ghost_tank
