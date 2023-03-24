#include <memory>
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "ghost_msgs/msg/depth_query.hpp"
#include "ghost_msgs/msg/depth_return.hpp"

namespace depth {

    class DepthCalculations : public rclcpp::Node {
        public:

            DepthCalculations();

        private:
            std::queue<ghost_msgs::msg::DepthQuery> DepthQueries;

            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Subscription<ghost_msgs::msg::DepthQuery>::SharedPtr depth_query_sub_;
            rclcpp::Publisher<ghost_msgs::msg::DepthReturn>::SharedPtr depth_return_pub_;

            void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
            void query_callback(const ghost_msgs::msg::DepthQuery::SharedPtr query);
    };
}