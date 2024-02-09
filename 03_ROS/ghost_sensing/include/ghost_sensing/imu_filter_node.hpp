#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
	#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ghost_sensing {

class IMUFilterNode : public rclcpp::Node {
public:
	IMUFilterNode();
private:
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_filtered_imu_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_viz_pub;
};

} // namespace ghost_sensing