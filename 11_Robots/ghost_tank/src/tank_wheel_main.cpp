#include <ghost_tank/tank_wheel_odom.hpp>

#include <iostream>

int main(int argc, char * argv[]){
    //rclcpp::init(argc, argv);
    auto tank = std::make_shared<ghost_tank::TankOdometry>();
    tank->updateEncoders(2.0, 5.0);
    std::cout << 10 << std::endl;
    std::vector<double> my_position = tank->getRobotPositionWorld();
    //rclcpp::shutdown();
    return 0;
}