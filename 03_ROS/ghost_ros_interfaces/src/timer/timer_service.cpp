#include <spawn.h>
#include "ghost_msgs/srv/check_timer.hpp"
#include "ghost_msgs/srv/start_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using ghost_msgs::srv::CheckTimer;
using ghost_msgs::srv::StartTimer;
using std::placeholders::_1;
using std::placeholders::_2;

class BagRecorderNode : public rclcpp::Node {
public:
	BagRecorderNode() :
		rclcpp::Node("timer_service") {
		declare_parameter<int>("max_timers", 10);
		max_timers_ = get_parameter("max_timers").as_int();

		this->start_timer_service_ = this->create_service<StartTimer>("start_timer",
		                                                              std::bind(&BagRecorderNode::StartTimerCallback, this, _1, _2));
		this->check_timer_service_ = this->create_service<CheckTimer>("check_timer",
		                                                              std::bind(&BagRecorderNode::CheckTimerCallback, this, _1, _2));
	};

	void StartTimerCallback(const std::shared_ptr<StartTimer::Request> req,
	                        std::shared_ptr<StartTimer::Response>      res);

	void CheckTimerCallback(const std::shared_ptr<CheckTimer::Request> req,
	                        std::shared_ptr<CheckTimer::Response>      res);

private:
	rclcpp::Service<StartTimer>::SharedPtr start_timer_service_;
	rclcpp::Service<CheckTimer>::SharedPtr check_timer_service_;
	int max_timers_;
	std::map<std::string, std::pair<rclcpp::Time, double> > timers_;
};

void BagRecorderNode::StartTimerCallback(const std::shared_ptr<StartTimer::Request> req,
                                         std::shared_ptr<StartTimer::Response>      res) {
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start");
}

void BagRecorderNode::CheckTimerCallback(const std::shared_ptr<CheckTimer::Request> req,
                                         std::shared_ptr<CheckTimer::Response>      res) {
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "check");
}

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	std::shared_ptr<BagRecorderNode> node = std::make_shared<BagRecorderNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
}