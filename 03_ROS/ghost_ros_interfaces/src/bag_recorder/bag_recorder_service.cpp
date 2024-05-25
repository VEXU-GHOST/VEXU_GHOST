/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include <spawn.h>
#include "ghost_msgs/srv/start_recorder.hpp"
#include "ghost_msgs/srv/stop_recorder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"

using ghost_msgs::srv::StartRecorder;
using ghost_msgs::srv::StopRecorder;
using std::placeholders::_1;
using std::placeholders::_2;

class BagRecorderNode : public rclcpp::Node
{
public:
  BagRecorderNode()
  : rclcpp::Node("bag_recorder_service")
  {
    declare_parameter<int64_t>("disk_space_required_kb", 250 * 1024);
    declare_parameter<int>("disk_check_period_s", 10);

    disk_space_required_ = get_parameter("disk_space_required_kb").as_int();
    disk_check_period_ = std::chrono::seconds(get_parameter("disk_check_period_s").as_int());

    this->start_service_ = this->create_service<StartRecorder>(
      "bag_recorder/start",
      std::bind(&BagRecorderNode::StartCallback, this, _1, _2));
    this->stop_service_ = this->create_service<StopRecorder>(
      "bag_recorder/stop",
      std::bind(&BagRecorderNode::StopCallback, this, _1, _2));
    disk_check_timer =
      this->create_wall_timer(
      disk_check_period_,
      std::bind(&BagRecorderNode::EnforceFreeDiskSpace, this));

    std::string startup_string;
    startup_string += "Service Created.\n";
    startup_string += "\tDisk Space Required (KB):    " + std::to_string(disk_space_required_) +
      "\n";
    startup_string += "\tDisk Check Period (Seconds): " +
      std::to_string(disk_check_period_.count());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), startup_string.c_str());
  }

  void StartCallback(StartRecorder::Request::SharedPtr req, StartRecorder::Response::SharedPtr res);
  void StopCallback(StopRecorder::Request::SharedPtr req, StopRecorder::Response::SharedPtr res);
  void SpawnRecorderProcess();
  void KillRecorderProcess();
  void EnforceFreeDiskSpace();

private:
  rclcpp::Service<StartRecorder>::SharedPtr start_service_;
  rclcpp::Service<StopRecorder>::SharedPtr stop_service_;
  bool recording_ = false;
  pid_t recorderPID_ = -1;
  unsigned long disk_space_required_;
  FILE * disk_check_output_;
  std::chrono::seconds disk_check_period_;
  rclcpp::TimerBase::SharedPtr disk_check_timer;
};

void BagRecorderNode::StartCallback(
  StartRecorder::Request::SharedPtr req,
  StartRecorder::Response::SharedPtr res)
{
  SpawnRecorderProcess();
}

void BagRecorderNode::StopCallback(
  StopRecorder::Request::SharedPtr req,
  StopRecorder::Response::SharedPtr res)
{
  KillRecorderProcess();
}


void BagRecorderNode::SpawnRecorderProcess()
{
  if (recording_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to start recorder; one already exists.");
    return;
  }

  recording_ = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting recorder.");

  recorderPID_ = fork();
  if (recorderPID_ == 0) {
    execl(
      "/bin/sh", "sh", "-c", "exec ros2 bag record -a -o ~/bags/$(date +%m_%d_%Y-%H_%M_%S)",
      NULL);
  }
}

void BagRecorderNode::KillRecorderProcess()
{
  if (!recording_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to stop recorder; none exist.");
    return;
  }

  recording_ = false;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping recorder.");

  if (recorderPID_ == -1) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "rclcpp"), "Recorder PID uninitialized! Unable to stop nonexistent process.");
    return;
  }

  std::string cmd_string = "kill -s INT " + std::to_string(recorderPID_);
  std::system(cmd_string.c_str());
}

void BagRecorderNode::EnforceFreeDiskSpace()
{
  if (!recording_) {
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Checking disk space.");
  disk_check_output_ = popen("df / -k --output=avail | grep -v \"Avail\"", "r");
  unsigned long free_space;
  fscanf(disk_check_output_, "%lu", &free_space);
  fclose(disk_check_output_);

  if (free_space < disk_space_required_) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Not enough disk space to safely continue recording.");
    KillRecorderProcess();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<BagRecorderNode> node = std::make_shared<BagRecorderNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
