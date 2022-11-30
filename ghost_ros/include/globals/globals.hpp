/*
 * Filename: globals
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Friday October 28th 2022 3:45:21 pm
 * Modified By: Maxx Wilson
 */

#ifndef GHOST_ROS__GLOBALS_HPP
#define GHOST_ROS__GLOBALS_HPP

#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/poses_2d.h"

namespace globals{

struct Pose2DWithCovariance
{
    pose_2d::Pose2D<float> pose;
    Eigen::Matrix<float, 3, 3> covariance;
};


    // // Sensor Data
    // std::vector<Eigen::Vector2f> point_cloud_;
    
    // Pose2DWithCovariance odom_;

    // // Robot State Estimation
    // Pose2DWithCovariance robot_pose_;

    // // Environment State Estimation
    // std::vector<Pose2DWithCovariance> robot_poses_;
    // std::vector<Pose2DWithCovariance> disc_locations;

    // Motor Outputs

extern std::string repo_base_dir;
extern bool run;

} // namespace globals

#endif // GHOST_ROS__GLOBALS_HPP