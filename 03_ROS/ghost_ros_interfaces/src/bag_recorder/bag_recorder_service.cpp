#include <spawn.h>
#include "ghost_msgs/srv/toggle_bag_recorder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"

using ghost_msgs::srv::ToggleBagRecorder;
using std::placeholders::_1;
using std::placeholders::_2;

class BagRecorderNode : public rclcpp::Node {
public:
	BagRecorderNode() :
		rclcpp::Node("toggle_bag_recorder_service") {
		this->service = this->create_service<ToggleBagRecorder>("toggle_bag_recorder",
		                                                        std::bind(&BagRecorderNode::ToggleRecorder, this, _1, _2));
	};

	void ToggleRecorder(const std::shared_ptr<ToggleBagRecorder::Request> req,
	                    std::shared_ptr<ToggleBagRecorder::Response>      res);

private:
	rclcpp::Service<ToggleBagRecorder>::SharedPtr service;
	bool recording = false;
	pid_t recorderPID{};
};

void BagRecorderNode::ToggleRecorder(const std::shared_ptr<ToggleBagRecorder::Request> req,
                                     std::shared_ptr<ToggleBagRecorder::Response>      res) {
	if(!recording){
		recording = true;
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting recorder.");

		recorderPID = fork();
		if(recorderPID == 0){
			execl("/bin/sh", "sh", "-c", "ros2 bag record -a", NULL);
		}
	}

	else{
		recording = false;
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping recorder.");
		std::system("kill -s INT $(ps -aux | grep '/bin/ros2 bag record -a' | grep -v grep | awk '{print $2;}')");
	}
}

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	std::shared_ptr<BagRecorderNode> node = std::make_shared<BagRecorderNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
}