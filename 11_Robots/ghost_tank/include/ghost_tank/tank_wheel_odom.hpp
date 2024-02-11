#pragma once
//#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <memory>

namespace ghost_tank {

class TankOdometry {
public:
	TankOdometry();
void setAngle(double angle) {
        m_Angle = angle;
    }

    void setXPosition(double x) {
        m_Xposition = x;
    }

    void setYPosition(double y) {
        m_Yposition = y;
    }
    
	void updateEncoders(double left_wheel, double right_wheel);

	std::vector<double> getRobotPositionWorld();
private:
	double m_Xposition;
	double m_Yposition;  
    double m_Angle;  
	double m_deltaAngle;
	const double m_ticks = 300;
	const double m_Wheeldistance = 5.5;
	const double m_circumference = 10.2101761242;
};

} // namespace ghost_tank