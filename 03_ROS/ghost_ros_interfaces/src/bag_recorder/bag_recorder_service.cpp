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
		rclcpp::Node("bag_recorder_service") {
		declare_parameter<int64_t>("disk_space_required_kb", 250 * 1024);
		declare_parameter<int>("disk_check_period_s", 10);

		disk_space_required_ = get_parameter("disk_space_required_kb").as_int();
		disk_check_period_ = std::chrono::seconds(get_parameter("disk_check_period_s").as_int());

		this->service_ = this->create_service<ToggleBagRecorder>("toggle_bag_recorder",
		                                                         std::bind(&BagRecorderNode::ToggleRecording, this, _1, _2));
		disk_check_timer = this->create_wall_timer(disk_check_period_, std::bind(&BagRecorderNode::EnforceFreeDiskSpace, this));

		std::string startup_string;
		startup_string += "Service Created.\n";
		startup_string += "\tDisk Space Required (KB):    " + std::to_string(disk_space_required_)       + "\n";
		startup_string += "\tDisk Check Period (Seconds): " + std::to_string(disk_check_period_.count());

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), startup_string.c_str());
	};

	void ToggleRecording(const std::shared_ptr<ToggleBagRecorder::Request> req,
	                     std::shared_ptr<ToggleBagRecorder::Response>      res);
	void SpawnRecorderProcess();
	void KillRecorderProcess();
	void EnforceFreeDiskSpace();

private:
	rclcpp::Service<ToggleBagRecorder>::SharedPtr service_;
	bool recording_ = false;
	pid_t recorderPID_ = -1;
	unsigned long disk_space_required_;
	FILE* disk_check_output_;
	std::chrono::seconds disk_check_period_;
	rclcpp::TimerBase::SharedPtr disk_check_timer;
};

void BagRecorderNode::ToggleRecording(const std::shared_ptr<ToggleBagRecorder::Request> req,
                                      std::shared_ptr<ToggleBagRecorder::Response>      res) {
	(!recording_) ? SpawnRecorderProcess() : KillRecorderProcess();
}

void BagRecorderNode::SpawnRecorderProcess() {
	recording_ = true;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting recorder.");

	recorderPID_ = fork();
	if(recorderPID_ == 0){
		execl("/bin/sh", "sh", "-c", "exec ros2 bag record -a -o ~/bags/$(date +%m_%d_%Y-%H_%M_%S)", NULL);
	}
}

void BagRecorderNode::KillRecorderProcess() {
	recording_ = false;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping recorder.");

	if(recorderPID_ == -1){
		RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Recorder PID uninitialized! Unable to stop nonexistent process.");
		return;
	}

	std::string cmd_string = "kill -s INT " + std::to_string(recorderPID_);
	std::system(cmd_string.c_str());
}

void BagRecorderNode::EnforceFreeDiskSpace() {
	if(!recording_){
		return;
	}

	RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Checking disk space.");
	disk_check_output_ = popen("df / -k --output=avail | grep -v \"Avail\"", "r");
	unsigned long free_space;
	fscanf(disk_check_output_, "%lu", &free_space);
	fclose(disk_check_output_);

	if(free_space < disk_space_required_){
		RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Not enough disk space to safely continue recording.");
		KillRecorderProcess();
	}
}

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	std::shared_ptr<BagRecorderNode> node = std::make_shared<BagRecorderNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
}