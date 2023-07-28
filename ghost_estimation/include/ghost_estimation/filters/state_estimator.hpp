#ifndef GHOST_ESTIMATION__STATE_ESTIMATOR_HPP
#define GHOST_ESTIMATION__STATE_ESTIMATOR_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "robot_localization/robot_localization_estimator.hpp"


// Implementation based on: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

namespace ghost_estimation
{
    class StateEstimator: public rclcpp::Node
    {
        
    };

    private:

    RobotLocalizationEstimator rle_;
}

#endif // GHOST_ESTIMATION__STATE_ESTIMATOR_HPP