#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BasicSwerveController : public rclcpp::Node{
public:
    BasicSwerveController(): Node("basic_swerve_controller"){
        steering_1_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/mod_1/setpoint", 10);
        steering_2_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/mod_2/setpoint", 10);
        steering_3_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/mod_3/setpoint", 10);
        wheel_1_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/wheel_1/setpoint", 10);
        wheel_2_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/wheel_2/setpoint", 10);
        wheel_3_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/wheel_3/setpoint", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10, 
            std::bind(&BasicSwerveController::cmd_vel_callback, this, _1)
            );
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        std::cout << "VELOCITY CALLBACK" << std::endl;
        auto x = msg->linear.x;
        auto y = msg->linear.y;
        auto w = msg->angular.z;
        float theta = 0.0;

        float vw1x = x - w*0.2286*sin(theta - 60.0/180.0*M_PI);
        float vw1y = y + w*0.2286*cos(theta - 60.0/180.0*M_PI);
        float vw2x = x - w*0.2286*sin(theta + 60.0/180.0*M_PI);
        float vw2y = y + w*0.2286*cos(theta + 90.0/180.0*M_PI);
        float vw3x = x - w*0.2286*sin(theta - 180.0/180.0*M_PI);
        float vw3y = y + w*0.2286*cos(theta - 180.0/180.0*M_PI);

        publish_twist_msg(steering_1_pub_, atan2(vw1y, vw1x)*180/M_PI, 0.0, 0.0);
        publish_twist_msg(steering_2_pub_, atan2(vw2y, vw2x)*180/M_PI, 0.0, 0.0);
        publish_twist_msg(steering_3_pub_, atan2(vw3y, vw3x)*180/M_PI, 0.0, 0.0);

        publish_twist_msg(wheel_1_pub_, 0.0, sqrt(vw1x*vw1x + vw1y*vw1y)/(2.75/2*0.0254/2/M_PI), 0.0);
        publish_twist_msg(wheel_2_pub_, 0.0, sqrt(vw2x*vw2x + vw2y*vw2y)/(2.75/2*0.0254/2/M_PI), 0.0);
        publish_twist_msg(wheel_3_pub_, 0.0, sqrt(vw3x*vw3x + vw3y*vw3y)/(2.75/2*0.0254/2/M_PI), 0.0);

        std::cout << atan2(vw1y, vw1x) << std::endl;
        std::cout << atan2(vw2y, vw2x) << std::endl;
        std::cout << atan2(vw3y, vw3x) << std::endl;
        std::cout << sqrt(vw1x*vw1x + vw1y*vw1y) << std::endl;
        std::cout << sqrt(vw2x*vw2x + vw2y*vw2y) << std::endl;
        std::cout << sqrt(vw3x*vw3x + vw3y*vw3y) << std::endl;
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicSwerveController>());
  rclcpp::shutdown();
  return 0;
}