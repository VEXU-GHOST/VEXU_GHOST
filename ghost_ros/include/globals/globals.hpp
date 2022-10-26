/*
 * Filename: globals
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday October 24th 2022 2:34:06 pm
 * Modified By: Maxx Wilson
 */

#ifndef GHOST_ROS__GLOBALS_HPP
#define GHOST_ROS__GLOBALS_HPP

#include <vector>

#include "Eigen/Dense"
#include <eigen3/Eigen/Core>
#include <rclcpp>

#include "shared/math/poses_2d.h"

namespace ghost{
namespace global{

struct Pose2DWithCovariance
{
    Pose2D pose;
    Eigen::matrix<float, 3, 3> covariance;
};


    // Sensor Data
    std::vector<Eigen::Vector2f> point_cloud_;
    
    Pose2DWithCovariance odom_;

    // Robot State Estimation
    Pose2DWithCovariance robot_pose;

    // Environment State Estimation
    std::vector<Pose2DWithCovariance> robot_
    std::vector<Pose2DWithCovariance> disc_locations;

    // Motor Outputs

};

} // global
} // ghost

#endif // GHOST_ROS__GLOBALS_HPP