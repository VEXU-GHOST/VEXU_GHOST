#include "ghost_estimation/filters/state_estimator.hpp"

using namespace std::chrono_literals;

namespace ghost_estimation
{
    class StateEstimator: public rclcpp::Node
    {
        public:
            StateEstimator(): Node("state_estimator")
            {
                publisher_ = this->create_publisher<ghost_msgs::msg::GhostRobotState>("/estimation/robot_state", 10);

                
                // Subscriptions
                laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan",
                    rclcpp::SensorDataQoS(),
                    std::bind(&StateEstimator::LaserCallback, this, _1));
            }
        
        private:

        
        // Predict Step
        // Estimated state. 0 is current state, 1 is predicted state
        // xk_0
        // xk_1
         
        // // Estimated covariance
        // Pk

        // // Prediction Matrix
        // Fk

        // // Control Matrix
        // Bk
        
        // // Control Vector
        // uk

        // // Noise
        // Qk

        // // Update Step
        // // Covariance of sensors (sensor noise)
        // Rk 

        // // Define Covariance Matrices
        // E0 = Pk * Hk^T
        // E1 = E0 - K * E0

        // // Define Kalman Gain K
        // K = E0 * (Hk^T * E0 + E1)^-1

        // // Calculate next state and next state covariance
        // xk_1 = xk_0 + K * (zk - Hk * xk_0)
        // Pk_1 = Pk_0 - K * Hk * Pk_0
    }
}