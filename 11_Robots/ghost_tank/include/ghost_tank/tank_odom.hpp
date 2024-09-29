#include <vector> 

namespace ghost_tank{
class TankOdometry
{
public:
    // Constructor
    TankOdometry();

    std::vector<double> getRobotWorldPosition(double left_encoders, double right_encoders);

    void resetEncoders();

private:
    // Position and orientation variables
    double prev_xpos = 0;
    double prev_ypos = 0;
    double current_xpos = 0; 
    double current_ypos = 0;
    double left_encoder = 0;
    double right_encoder = 0;
    double theta = 0;
    double dl;
    double dr;
    double dcenter;
   
    const double ticks = 300;       // get total number of ticks on encoder per revolution
    const double circumference = 10.2101761242 ;  // circumference of wheels in inches
    const double Rw = 5.5;

};
} //namespace ghost_tank