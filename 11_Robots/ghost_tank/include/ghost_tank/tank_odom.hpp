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
        return m_cur_pos;
    };

    //Eigen::Vector2d getRobotWorldPosition(double left_encoders, double right_encoders);
    Eigen::Vector3d update(std::vector<long> l_wheel_pos, std::vector<long> r_wheel_pos);


    Eigen::Vector3d resetPose() {
        return setPose({0,0,0});
    }

    Eigen::Vector3d setPose(Eigen::Vector3d p) {
        m_cur_pos = p;
        return p;
    };


private:
    std::vector<long> m_prev_l_ticks;
    std::vector<long> m_prev_r_ticks;

    Eigen::Vector3d m_cur_pos;

    double m_meters_per_tick;
    double m_wheelbase; 
};
} //namespace ghost_tank