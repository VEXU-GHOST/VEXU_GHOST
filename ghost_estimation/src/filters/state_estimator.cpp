#include "ghost_estimation/filters/state_estimator.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace ghost_estimation
{
    StateEstimator::StateEstimator() : Node("state_estimator")
    {

        // LoadROSParams();
        
        // Publishers
        state_publisher_ = this->create_publisher<ghost_msgs::msg::GhostRobotState>("/estimation/robot_state", 10);

        
        // Subscriptions
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&StateEstimator::LaserCallback, this, _1));

        encoder_sub_ = this->create_subscription<ghost_msgs::msg::V5SensorUpdate>(
            "/v5/sensor_update",
            10,
            std::bind(&StateEstimator::EncoderCallback, this, _1));
    }

    void StateEstimator::LoadROSParams(){
    }

    void StateEstimator::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        last_laser_msg_ = msg;
    }

    void StateEstimator::EncoderCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
        last_encoder_msg_ = msg;
    }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_estimation::StateEstimator>());
  rclcpp::shutdown();
  return 0;
}