#include <ghost_tank/tank_wheel_odom.hpp>
//#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>

namespace ghost_tank {


TankOdometry::TankOdometry(){
	std::cout << "Constructor" << std::endl;
	//m_Xposition = 0;
	//m_Yposition = 0;   
   // m_Angle = 0;  
	//m_deltaAngle = 0;
}

void TankOdometry::updateEncoders(double left_wheel, double right_wheel){
	std::cout << "updateEncoders" << std::endl;

	double prevencoderleft = 0;
	double prevencoderright = 0;
	//const double ticks = 300;       // get total number of ticks on encoder per revolution
	//const double circumference = 10.2101761242 ;  // circumference of wheels in inches
	//const double Rwheel = 5.5;       // get distance between wheel and center of robot (should be same for both sides?)
	double Xinitial = 0;        // initial position of robot
	double Yinitial = 0;        // initial position of robot
	double initialAngle = 0;    // initial angle of robot
	double prevX = 0;
	double prevY = 0;
	//m_Xposition = Xinitial;
	//m_Yposition = Yinitial;
	//m_Angle = initialAngle;

	//while(true){
	// for initial encoder reading
	if((m_Xposition == Xinitial) && (m_Yposition == Yinitial) ){
		prevX = m_Xposition;
		prevY = m_Yposition;
	}

	double encoderleft = left_wheel; // get value from encoder
	double encoderright = right_wheel; // get value from encoder
	double distanceleft = (((encoderleft - prevencoderleft) / m_ticks) * m_circumference);   // getting distance traveled by left encoder
	double distanceright = (((encoderright - prevencoderright) / m_ticks) * m_circumference);  // distance traveled by right encoder

	prevencoderleft = encoderleft;
	prevencoderright = encoderright;


	m_deltaAngle = ((distanceright - distanceleft) / 2 * m_Wheeldistance);    // find change in angle

	// for straight motion, both distances should be equal?
	if(distanceleft == distanceright){
		m_Xposition += distanceleft * cos(m_Angle);   // add x component of distance to previous position
		m_Yposition += distanceright * sin(m_Angle);  // add y component of distance to previous position
		m_Angle += m_deltaAngle;
	}
	else if(distanceleft == -distanceright){   //turning in place
	m_Xposition = m_Xposition;
	m_Yposition = m_Yposition;
	m_deltaAngle = ((distanceright)/m_Wheeldistance);
    m_Angle += m_deltaAngle;
	}

	else{
		double distance = ((distanceleft + distanceright) / 2); // distance traveled by robot
		double deltaX = (distance * cos(m_deltaAngle));
		double deltaY = (distance * sin(m_deltaAngle));
		//new_xpos = (m_Xposition*cos(deltaAngle) - m_Yposition*sin(deltaAngle)+ distance)
		// change to global coordinates
		m_Xposition += deltaX;
		m_Yposition += deltaY;
		m_Angle += m_deltaAngle;
		// probably need linear algebra here or something to make sure shift is relative to world
	}
	
	
}
std::vector<double> TankOdometry::getRobotPositionWorld(){
	std::cout << "getRobotPositionWorld" << std::endl;
	return {m_Xposition, m_Yposition, m_Angle};
}

} // namespace ghost_tank