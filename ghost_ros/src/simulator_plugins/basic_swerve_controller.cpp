#ifndef GHOST_ROS__BASIC_SWERVE_CONTROLLER_HPP
#define GHOST_ROS__BASIC_SWERVE_CONTROLLER_HPP

#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

class BasicSwerveController : public rclcpp::Node{
public:
    BasicSwerveController(): Node("basic_swerve_controller"){
        steering_1_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/steering_1", 10);
        steering_2_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/steering_2", 10);
        steering_3_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/steering_3", 10);
        wheel_1_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/driveshaft_1", 10);
        wheel_2_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/driveshaft_2", 10);
        wheel_3_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/driveshaft_3", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, &BasicSwerveController::cmd_vel_callback);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        auto x = msg->linear.x;
        auto y = msg->linear.y;
        auto w = msg->angular.z;
        float theta = 0.0;

        float vw1x = x - w*0.2286*sin(theta + 30.0/180.0*M_PI);
        float vw1y = y + w*0.2286*cos(theta + 30.0/180.0*M_PI);
        float vw2x = x - w*0.2286*sin(theta + 150.0/180.0*M_PI);
        float vw2y = y + w*0.2286*cos(theta + 150.0/180.0*M_PI);
        float vw3x = x - w*0.2286*sin(theta - 90.0/180.0*M_PI);
        float vw3y = y + w*0.2286*cos(theta - 90.0/180.0*M_PI);

        publish_twist_msg(steering_1_pub_, atan2(vw1y, vw1x), 0.0, 0.0);
        publish_twist_msg(steering_2_pub_, atan2(vw2y, vw2x), 0.0, 0.0);
        publish_twist_msg(steering_3_pub_, atan2(vw3y, vw3x), 0.0, 0.0);

        publish_twist_msg(wheel_1_pub_, 0.0, sqrt(vw1x*vw1x + vw1y*vw1y), 0.0);
        publish_twist_msg(wheel_2_pub_, 0.0, sqrt(vw2x*vw2x + vw2y*vw2y), 0.0);
        publish_twist_msg(wheel_3_pub_, 0.0, sqrt(vw3x*vw3x + vw3y*vw3y), 0.0);
    }
private:

    void publish_twist_msg(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub, float x, float y, float z){
        geometry_msgs::msg::Vector3 msg{};
        msg.x = x;
        msg.y = y;
        msg.z = z;
        pub->publish(msg);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr steering_1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr steering_2_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr steering_3_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_2_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_3_pub_;
    
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

#endif //GHOST_ROS__BASIC_SWERVE_CONTROLLER_HPP