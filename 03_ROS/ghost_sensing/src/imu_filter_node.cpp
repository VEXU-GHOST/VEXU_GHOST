#include <iostream>
#include <ghost_sensing/imu_filter_node.hpp>

using std::placeholders::_1;

namespace ghost_sensing {

IMUFilterNode::IMUFilterNode() :
	rclcpp::Node("imu_filter_node"){
	m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
		"/camera/imu", 10, std::bind(&IMUFilterNode::imu_callback, this, _1));

	m_filtered_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
		"sensing/imu/filtered", 10);

	m_viz_pub = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
		marker_array_topic,
		10);
}

void IMUFilterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
	std::cout << "Running!" << std::endl;

	// sensor_msgs::msg::Imu filtered_msg{};
	// m_filtered_imu_pub->publish(filtered_msg);

	visualization_msgs::msg::MarkerArray viz_msg{};

	visualization_msgs::msg::Marker gyro_marker{};

	visualization_msgs::msg::Marker accel_marker{};
}

} // namespace ghost_sensing

int main(int argc,
         char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_sensing::IMUFilterNode>());
	rclcpp::shutdown();
	return 0;
}