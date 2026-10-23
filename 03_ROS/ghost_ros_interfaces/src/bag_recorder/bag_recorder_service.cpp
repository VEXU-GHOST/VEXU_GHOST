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

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <vector>
#include "ghost_msgs/srv/start_recorder.hpp"
#include "ghost_msgs/srv/stop_recorder.hpp"
#include "std_msgs/msg/int64.hpp"
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
    process_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&BagRecorderNode::CheckRecorderProcesses, this));

    // Create subscription to /io/buttons/bag and publisher to /io/leds/bag
    button_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
      "/io/buttons/bag", 10,
      std::bind(&BagRecorderNode::ButtonCallback, this, _1));
    led_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/io/leds/bag", 10);

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
  void CheckRecorderProcesses();
  void LowerRecorderPriority(pid_t pid);
  void EnforceFreeDiskSpace();
  void ButtonCallback(const std_msgs::msg::Int64::SharedPtr msg);
  void PublishLedStatus(int64_t status);

private:
  rclcpp::Service<StartRecorder>::SharedPtr start_service_;
  rclcpp::Service<StopRecorder>::SharedPtr stop_service_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr button_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr led_publisher_;


  bool recording_ = false;
  pid_t recorderPID_ = -1;
  std::vector<pid_t> finalizing_pids_;
  rclcpp::TimerBase::SharedPtr process_check_timer_;
  unsigned long disk_space_required_;
  FILE * disk_check_output_;
  std::chrono::seconds disk_check_period_;
  rclcpp::TimerBase::SharedPtr disk_check_timer;
  int64_t last_button_state_ = -1;  // Initialized to an invalid state
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
  CheckRecorderProcesses();
  if (recording_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to start recorder; one already exists.");
    return;
  }

  recorderPID_ = fork();
  if (recorderPID_ < 0) {
    RCLCPP_ERROR(get_logger(), "Unable to start recorder: %s", std::strerror(errno));
    return;
  }
  if (recorderPID_ == 0) {
    // One unsplit SQLite file: file-mode compression only starts after SIGINT closes it.
    execl(
      "/bin/sh", "sh", "-c",
      "R=$(cat \"$VEXU_HOME/robotname\" 2>/dev/null || echo unknown_robot); "
      "D=$(date +%m_%d_%Y); N=1; "
      "while [ -e \"$HOME/bags/${R}_${D}_${N}\" ]; do N=$((N+1)); done; "
      "exec ros2 bag record -a --storage sqlite3 "
      "--max-bag-size 0 --max-bag-duration 0 "
      "--compression-mode file --compression-format zstd --compression-threads 1 "
      "-o \"$HOME/bags/${R}_${D}_${N}\"",
      NULL);
    _exit(127);
  }

  recording_ = true;
  PublishLedStatus(1);
  RCLCPP_INFO(get_logger(), "Recording uncompressed; lossless Zstd compression will run after stop.");
}

void BagRecorderNode::KillRecorderProcess()
{
  if (!recording_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to stop recorder; none exist.");
    return;
  }

  if (recorderPID_ == -1) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "rclcpp"), "Recorder PID uninitialized! Unable to stop nonexistent process.");
    return;
  }

  // Compression workers already exist in Humble. Lower every thread, not just the
  // main thread, before SIGINT starts finalization. New match recorders retain normal priority.
  LowerRecorderPriority(recorderPID_);
  if (kill(recorderPID_, SIGINT) != 0) {
    RCLCPP_ERROR(get_logger(), "Unable to stop recorder: %s", std::strerror(errno));
    return;
  }

  finalizing_pids_.push_back(recorderPID_);
  recorderPID_ = -1;
  recording_ = false;
  RCLCPP_INFO(get_logger(), "Recording stopped; finalizing and compressing the bag in the background.");

  PublishLedStatus(0);
}

void BagRecorderNode::LowerRecorderPriority(pid_t pid)
{
  const std::string task_path = "/proc/" + std::to_string(pid) + "/task";
  DIR * tasks = opendir(task_path.c_str());
  if (!tasks) {
    RCLCPP_WARN(get_logger(), "Could not lower compression priority: %s", std::strerror(errno));
    return;
  }

  while (const auto * entry = readdir(tasks)) {
    char * end = nullptr;
    const long tid = std::strtol(entry->d_name, &end, 10);
    if (tid <= 0 || *end != '\0') {
      continue;
    }
    errno = 0;
    const int priority = getpriority(PRIO_PROCESS, tid);
    if (errno == 0 && priority < 10 && setpriority(PRIO_PROCESS, tid, 10) != 0 &&
      errno != ESRCH)
    {
      RCLCPP_WARN(get_logger(), "Could not lower compression thread priority: %s", std::strerror(errno));
    }
  }
  closedir(tasks);
}

void BagRecorderNode::CheckRecorderProcesses()
{
  int status = 0;
  if (recording_ && waitpid(recorderPID_, &status, WNOHANG) == recorderPID_) {
    RCLCPP_ERROR(get_logger(), "Recorder exited before a stop request (wait status %d).", status);
    recorderPID_ = -1;
    recording_ = false;
    PublishLedStatus(0);
  }

  for (auto it = finalizing_pids_.begin(); it != finalizing_pids_.end();) {
    const pid_t result = waitpid(*it, &status, WNOHANG);
    if (result == 0 || (result < 0 && errno == EINTR)) {
      ++it;
      continue;
    }
    if (result > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0) {
      RCLCPP_INFO(get_logger(), "Bag finalization/compression process %d finished.", *it);
    } else {
      RCLCPP_ERROR(get_logger(), "Bag finalization/compression process %d failed; inspect its bag and logs.", *it);
    }
    it = finalizing_pids_.erase(it);
  }
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

// Subscription callback for /io/buttons/bag
void BagRecorderNode::ButtonCallback(const std_msgs::msg::Int64::SharedPtr msg)
{
  if (msg->data != last_button_state_) {
    RCLCPP_INFO(this->get_logger(), "Button state changed from %ld to %ld", last_button_state_, msg->data);
    
    if (msg->data == 1) {
      SpawnRecorderProcess();
    } else if (msg->data == 0) {
      KillRecorderProcess();
    } else {
      RCLCPP_WARN(this->get_logger(), "Received unknown button command: %ld", msg->data);
    }
    
    // Update the last button state to the current value
    last_button_state_ = msg->data;
  }
}

// Helper function to publish LED status on /io/leds/bag
void BagRecorderNode::PublishLedStatus(int64_t status)
{
  std_msgs::msg::Int64 led_msg;
  led_msg.data = status;
  led_publisher_->publish(led_msg);
  RCLCPP_INFO(this->get_logger(), "Published LED status: %ld", led_msg.data);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<BagRecorderNode> node = std::make_shared<BagRecorderNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
