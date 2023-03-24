#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ghost_msgs/msg/depth_query.hpp"
#include "ghost_msgs/msg/depth_return.hpp"

using std::placeholders::_1;
using ghost_msgs::msg::DepthQuery;
using ghost_msgs::msg::DepthReturn;


namespace depth
{
  class DepthCalculations : public rclcpp::Node {
    public:
      DepthCalculations() : Node("depth_calculations") {

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 
          10, 
          std::bind(&DepthCalculations::scan_callback, this, _1));

        depth_query_sub_ = this->create_subscription<DepthQuery>(
          "depth/queries", 
          10, 
          std::bind(&DepthCalculations::query_callback, this, _1));

        depth_return_pub_ = this->create_publisher<DepthReturn>("depth/returns", 10);

      }

      private:
        std::queue<DepthQuery> DepthQueries;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<DepthQuery>::SharedPtr depth_query_sub_;
        rclcpp::Publisher<DepthReturn>::SharedPtr depth_return_pub_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
        {
          while (!DepthQueries.empty()) {
            auto query = DepthQueries.front();
            DepthQueries.pop();

            auto inRange = [scan](float value) {
              return (scan->angle_min <= value) && (value <= scan->angle_max);
            };

            if (inRange(query.min_angle) && inRange(query.max_angle)) {
              int start = (query.min_angle-scan->angle_min)/(scan->angle_increment);
              int end = (query.max_angle-scan->angle_min)/(scan->angle_increment);
              float rangeSum = 0;
              int currentIndex = start;
              while (currentIndex <= end) {
                rangeSum += (scan->ranges)[currentIndex];
                currentIndex++;
              }

              auto depthReturn = DepthReturn();

              depthReturn.min_angle = query.min_angle;
              depthReturn.max_angle = query.max_angle;
              depthReturn.distance = (rangeSum/(end-start));

              depth_return_pub_->publish(depthReturn);
            }

          }
        }
        void query_callback(const DepthQuery::SharedPtr query)
        {
          DepthQueries.push(*query);
        }
  };
} 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<depth::DepthCalculations>());
    rclcpp::shutdown();
    return 0;
}