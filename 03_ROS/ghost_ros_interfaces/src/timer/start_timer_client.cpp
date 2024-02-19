#include "ghost_msgs/srv/start_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_timer_client");

	rclcpp::Client<ghost_msgs::srv::StartTimer>::SharedPtr start_timer_client =
		node->create_client<ghost_msgs::srv::StartTimer>("start_timer");
	auto request = std::make_shared<ghost_msgs::srv::StartTimer::Request>();

	while(!start_timer_client->wait_for_service(1s)){
		if(!rclcpp::ok()){
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
	}

	auto response = start_timer_client->async_send_request(request);

	if(rclcpp::spin_until_future_complete(node, response) != rclcpp::FutureReturnCode::SUCCESS){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service start_timer");
	}

	rclcpp::shutdown();
}