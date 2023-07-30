#ifndef GHOST_ESTIMATION__STATE_ESTIMATOR_HPP
#define GHOST_ESTIMATION__STATE_ESTIMATOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <stdio.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_localization/ekf.hpp"

#include "ghost_msgs/msg/ghost_robot_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


namespace ghost_estimation{

    class StateEstimator : public rclcpp::Node{
        public:
        
        // Constructor
        StateEstimator();

        // Publishers
        rclcpp::Publisher<ghost_msgs::msg::GhostRobotState>::SharedPtr state_publisher_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Subscription<ghost_msgs::msg::V5SensorUpdate>::SharedPtr encoder_sub_;

        private:
        void LoadROSParams();
        void EncoderCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg);
        void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        sensor_msgs::msg::LaserScan::SharedPtr last_laser_msg_;
        ghost_msgs::msg::V5SensorUpdate::SharedPtr last_encoder_msg_;

    };
}

#endif // GHOST_ESTIMATION__STATE_ESTIMATOR_HPP