#include "tank_odom.hpp"
#include <vector>
#include <cmath>


namespace ghost_tank{

    std::vector<double> TankOdometry::getRobotWorldPosition(double in_left_encoder, double in_right_encoder){

    double dl = (in_left_encoder - left_encoder)/ticks * circumference;
    double dr = (in_right_encoder - left_encoder)/ticks * circumference;
    double dcenter = (dl+dr)/2.0;
    theta += (dr - dl)/ (2.0*Rw);

    
    current_xpos = prev_xpos + dcenter*std::cos(theta);
    current_ypos = prev_ypos + dcenter*std::sin(theta);

    left_encoder = in_left_encoder;
    right_encoder = in_right_encoder;
    
    prev_xpos = current_xpos;
    prev_ypos = current_ypos;

    return {current_xpos, current_ypos, theta};
    }

    void TankOdometry::resetEncoders(){
        double prev_xpos = 0;
        double prev_ypos = 0;
        double current_xpos = 0; 
        double current_ypos = 0;
        double left_encoder = 0;
        double right_encoder = 0;
        double theta = 0;
    }

}//namespace ghost_tank
