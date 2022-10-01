#ifndef GHOST_ROS_MAIN_HPP
#define GHOST_ROS_MAIN_HPP

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class SimulatedRobotNode : public rclcpp::Node{
public:
    SimulatedRobotNode(std::string node_name);
    ~SimulatedRobotNode();

    
private:

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
};

#endif //GHOST_ROS_MAIN_HPP