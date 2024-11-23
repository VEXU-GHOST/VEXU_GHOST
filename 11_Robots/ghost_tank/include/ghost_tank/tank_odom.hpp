#include <vector> 
#include "eigen3/Eigen/Geometry"
#pragma once


namespace ghost_tank{

class TankOdometry
{
public:
    // Constructor
    TankOdometry(int ticks_per_rotation, double wheel_radius_m, double wheelbase_m);

    Eigen::Vector3d getPose() {
        return cur_pos;
    };

    //Eigen::Vector2d getRobotWorldPosition(double left_encoders, double right_encoders);
    Eigen::Vector3d update(std::vector<long> l_wheel_pos, std::vector<long> r_wheel_pos);


    Eigen::Vector3d resetPose() {
        return setPose({0,0,0});
    }

    Eigen::Vector3d setPose(Eigen::Vector3d p) {
        cur_pos = p;
        return p;
    };


private:
    std::vector<long> prev_l_ticks;
    std::vector<long> prev_r_ticks;
    double prev_angle;

    Eigen::Vector3d cur_pos;

    double meters_per_tick;
    double wheelbase; 
};
} //namespace ghost_tank