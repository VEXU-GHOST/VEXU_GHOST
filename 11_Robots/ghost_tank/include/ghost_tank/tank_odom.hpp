#include <vector> 
#include "eigen3/Eigen/Geometry"
#pragma once


namespace ghost_tank{
typedef struct { 
    Eigen::Vector2d dist;
    double angle;
} poseish; // pose, derivative, double derivatvie (i forgot what its called)

using Eigen::Vector3d;
class TankOdometry
{
public:
    // Constructor
    TankOdometry(int ticks_per_rotation, double wheel_radius_m, double wheelbase_m);

    Vector3d getPose() {
        return cur_pos;
    };

    //Eigen::Vector2d getRobotWorldPosition(double left_encoders, double right_encoders);
    Vector3d update(std::vector<long> l_wheel_pos, std::vector<long> r_wheel_pos, double angle_rad);


    Vector3d resetPose() {
        return setPose({0,0,0});
    }

    Vector3d setPose(Vector3d p) {
        cur_pos = p;
        return p;
    };


private:
//    // Position and orientation variables
    //double prev_xpos = 0;
    //double prev_ypos = 0;
    //double current_xpos = 0; 
    //double current_ypos = 0;
    //double left_encoder = 0;
    //double right_encoder = 0;
    //double theta = 0;
    //double dl;
    //double dr;
    //double dcenter;

    std::vector<long> prev_l_ticks;
    std::vector<long> prev_r_ticks;
    double prev_angle;

Eigen::Vector3d cur_pos;
    //Eigen::Vector2d m_odom_loc;
    //double m_odom_angle;
   

    double meters_per_tick;
    double wheelbase; 
    //const double ticks = 300;       // get total number of ticks on encoder per revolution
    //const double circumference = 10.2101761242 ;  // circumference of wheels in inches
    //const double Rw = 5.5;

};
} //namespace ghost_tank