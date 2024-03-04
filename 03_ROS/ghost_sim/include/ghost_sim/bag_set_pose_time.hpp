#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace bag_set_pose_time {

class BagSetPoseTime : public rclcpp::Node {
public:
	/// Constructor
	BagSetPoseTime();

private:
	void BagSetPoseTimeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rviz_set_pose_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_time_pub_;
};

}// namespace bag_set_pose