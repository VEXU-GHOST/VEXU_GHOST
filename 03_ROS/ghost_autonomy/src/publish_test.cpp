#include "ghost_autonomy/publish_test.hpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;
TestPublisher::TestPublisher() :
		rclcpp::Node("run_tree_node"){
		robot_pub_ = create_publisher<ghost_msgs::msg::V5SensorUpdate>(
				"/v5/sensor_update",
				10);
		ghost_msgs::msg::V5SensorUpdate sensor_update_msg{};
		auto curr_ros_time = get_clock()->now();
		sensor_update_msg.header.stamp = curr_ros_time;
		robot_pub_->publish(sensor_update_msg);
}

int main(int argc, char *argv[]){
		rclcpp::init(argc, argv);
		auto node = std::make_shared<TestPublisher>();
		rclcpp::spin(node);

		rclcpp::shutdown();
		return 0;
}