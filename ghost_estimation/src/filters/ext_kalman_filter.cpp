#include "ext_kalman_filter.hpp"

using namespace std::chrono_literals;

namespace ghost_estimation
{
    class ExtKalmanFilter: public rclcpp::Node
    {
        public:
            ExtKalmanFilter()
            : Node("ext_kalman_filter"), count_(0)
            {

            }
        
        private:
        // Predict Step
        // Estimated state. 0 is current state, 1 is predicted state
        xk_0
        xk_1
         
        // Estimated covariance
        Pk

        // Prediction Matrix
        Fk

        // Control Matrix
        Bk
        
        // Control Vector
        uk

        // Noise
        Qk

        // Update Step
        // Covariance of sensors (sensor noise)
        Rk 

        // Define Covariance Matrices
        E0 = Pk * Hk^T
        E1 = E0 - K * E0

        // Define Kalman Gain K
        K = E0 * (Hk^T * E0 + E1)^-1

        // Calculate next state and next state covariance
        xk_1 = xk_0 + K * (zk - Hk * xk_0)
        Pk_1 = Pk_0 - K * Hk * Pk_0
    }
}