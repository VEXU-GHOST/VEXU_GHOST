#include "ghost_msgs/srv/check_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("check_timer_client");

	rclcpp::Client<ghost_msgs::srv::CheckTimer>::SharedPtr check_timer_client =
		node->create_client<ghost_msgs::srv::CheckTimer>("check_timer");
	auto request = std::make_shared<ghost_msgs::srv::CheckTimer::Request>();

	while(!check_timer_client->wait_for_service(1s)){
		if(!rclcpp::ok()){
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
	}

	auto response = check_timer_client->async_send_request(request);

	if(rclcpp::spin_until_future_complete(node, response) != rclcpp::FutureReturnCode::SUCCESS){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service check_timer");
	}

	rclcpp::shutdown();
}