/*
 * Filename: ground_truth_pose
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday October 24th 2022 2:24:18 pm
 * Modified By: Maxx Wilson
 */

#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "Eigen/Dense"
#include <eigen3/Eigen/Core>

using namespace std::chrono_literals;
using std::placeholders::_1;

class BasicSwerveController : public rclcpp::Node{
public:
    BasicSwerveController(): Node("basic_swerve_controller"){
        steering_1_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/motors/mod_1/setpoint", 10);

        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states",
            10, 
            std::bind(&BasicSwerveController::model_states_callback, this, _1)
            );

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Velocity Callback");
        auto jx = msg->linear.x;
        auto jy = msg->linear.y;
        auto jw = -msg->angular.z;

        if(abs(jx) < 0.05){
            jx = 0;
        }
        if(abs(jy) < 0.05){
            jy = 0;
        }
        if(abs(jw) < 0.05){
            jw = 0;
        }
        
        const float max_wheel_rpm = 500.0;
        const float wheel_radius = 1.375;
        const float rpm_to_ms = 2*3.14159*wheel_radius*0.0254/60.0;
        const float rpm_to_rads = rpm_to_ms/0.2286;

        RCLCPP_INFO(this->get_logger(), "\n\n\n");
        RCLCPP_INFO(this->get_logger(), "jx: %lf", jx);
        RCLCPP_INFO(this->get_logger(), "jy: %lf", jy);
        RCLCPP_INFO(this->get_logger(), "jw: %lf", jw);

        jx *= max_wheel_rpm*rpm_to_ms;
        jy *= max_wheel_rpm*rpm_to_ms;
        jw *= max_wheel_rpm*rpm_to_rads;

        RCLCPP_INFO(this->get_logger(), "-----------");
        RCLCPP_INFO(this->get_logger(), "jx: %lf", jx);
        RCLCPP_INFO(this->get_logger(), "jy: %lf", jy);
        RCLCPP_INFO(this->get_logger(), "jw: %lf", jw);

        float jx_bl = cos(theta_)*jx - sin(theta_)*jy;
        float jy_bl = sin(theta_)*jx + cos(theta_)*jy;

        float vw1x = jx_bl - jw*sin(-60.0/180.0*M_PI);
        float vw1y = jy_bl + jw*cos(-60.0/180.0*M_PI);
        float vw2x = jx_bl - jw*sin(90.0/180.0*M_PI);
        float vw2y = jy_bl + jw*cos(90.0/180.0*M_PI);
        float vw3x = jx_bl - jw*sin(-180.0/180.0*M_PI);
        float vw3y = jy_bl + jw*cos(-180.0/180.0*M_PI);

        RCLCPP_INFO(this->get_logger(), "vw1x: %lf", vw1x);
        RCLCPP_INFO(this->get_logger(), "vw1y: %lf", vw1y);
        RCLCPP_INFO(this->get_logger(), "vw2x: %lf", vw2x);
        RCLCPP_INFO(this->get_logger(), "vw2y: %lf", vw2y);
        RCLCPP_INFO(this->get_logger(), "vw3x: %lf", vw3x);
        RCLCPP_INFO(this->get_logger(), "vw3y: %lf", vw3y);

        publish_twist_msg(steering_1_pub_, atan2(vw1y, vw1x)*180/M_PI, 0.0, 0.0);
        publish_twist_msg(steering_2_pub_, atan2(vw2y, vw2x)*180/M_PI, 0.0, 0.0);
        publish_twist_msg(steering_3_pub_, atan2(vw3y, vw3x)*180/M_PI, 0.0, 0.0);

        publish_twist_msg(wheel_1_pub_, 0.0, sqrt(vw1x*vw1x + vw1y*vw1y)/rpm_to_ms, 0.0);
        publish_twist_msg(wheel_2_pub_, 0.0, sqrt(vw2x*vw2x + vw2y*vw2y)/rpm_to_ms, 0.0);
        publish_twist_msg(wheel_3_pub_, 0.0, sqrt(vw3x*vw3x + vw3y*vw3y)/rpm_to_ms, 0.0);

        RCLCPP_INFO(this->get_logger(), "%lf", atan2(vw1y, vw1x));
        RCLCPP_INFO(this->get_logger(), "%lf", atan2(vw2y, vw2x));
        RCLCPP_INFO(this->get_logger(), "%lf", atan2(vw3y, vw3x));
        RCLCPP_INFO(this->get_logger(), "%lf", sqrt(vw1x*vw1x + vw1y*vw1y));
        RCLCPP_INFO(this->get_logger(), "%lf", sqrt(vw2x*vw2x + vw2y*vw2y));
        RCLCPP_INFO(this->get_logger(), "%lf", sqrt(vw3x*vw3x + vw3y*vw3y));
        RCLCPP_INFO(this->get_logger(), "--");

    }

    void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Model States Callback");
        int swerve_model_index = -1;
        for(int i = 0; i < msg->name.size(); i++){
            if(msg->name[i] == "swerve_pid"){
                swerve_model_index = i;
            }
        }
        if(swerve_model_index == -1){
            RCLCPP_WARN(this->get_logger(), "Swerve Gazebo Model not found in /ModelStates topic");
        }
        else{
            x_ = msg->pose[swerve_model_index].position.x;
            y_ = msg->pose[swerve_model_index].position.y;
            float q1 = msg->pose[swerve_model_index].orientation.x;
            float q2 = msg->pose[swerve_model_index].orientation.x;
            float q3 = msg->pose[swerve_model_index].orientation.z;
            float q4 = msg->pose[swerve_model_index].orientation.w;
            theta_ = acos(1-2*(q2*q2 + q3*q3));
            // RCLCPP_INFO(this->get_logger(), "x: %lf", x_);
            // RCLCPP_INFO(this->get_logger(), "y: %lf", y_);
            // RCLCPP_INFO(this->get_logger(), "theta: %lf", theta_);
            // RCLCPP_INFO(this->get_logger(), "-------------------");
            // RCLCPP_INFO(this->get_logger(), "x: %lf", q1);
            // RCLCPP_INFO(this->get_logger(), "y: %lf", q2);
            // RCLCPP_INFO(this->get_logger(), "z: %lf", q3);
            // RCLCPP_INFO(this->get_logger(), "w: %lf", q4);
        }
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
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;

    // Member Variables
    float x_;
    float y_;
    float theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPose>());
  rclcpp::shutdown();
  return 0;
}