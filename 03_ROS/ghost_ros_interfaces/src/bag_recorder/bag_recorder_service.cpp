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
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <random>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "ghost_msgs/srv/start_recorder.hpp"
#include "ghost_msgs/srv/stop_recorder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

extern char ** environ;
using ghost_msgs::srv::StartRecorder;
using ghost_msgs::srv::StopRecorder;
using Clock = std::chrono::steady_clock;
using std::placeholders::_1;
using std::placeholders::_2;

class BagRecorderNode : public rclcpp::Node
{
public:
  BagRecorderNode() : Node("bag_recorder_service")
  {
    const char * home = std::getenv("HOME");
    const auto directory = declare_parameter<std::string>(
      "bag_directory", home ? std::string(home) + "/bags" : "");
    min_free_kb_ = declare_parameter<int64_t>("disk_space_required_kb", 5 * 1024 * 1024);
    resume_margin_kb_ = declare_parameter<int64_t>("disk_resume_margin_kb", 1024 * 1024);
    const auto period = declare_parameter<int64_t>("disk_check_period_s", 5);
    max_size_ = declare_parameter<int64_t>("max_bag_size_bytes", 1024 * 1024 * 1024);
    max_duration_ = declare_parameter<int64_t>("max_bag_duration_s", 300);
    requested_ = declare_parameter<bool>("autostart", false);
    const bool buttons = declare_parameter<bool>("enable_button_control", true);
    if (directory.empty() || min_free_kb_ < 0 || resume_margin_kb_ < 0 ||
      period <= 0 || max_size_ <= 0 || max_duration_ <= 0)
    {
      throw std::invalid_argument("Invalid bag recorder directory, disk reserve, or timing parameters");
    }
    directory_ = std::filesystem::absolute(directory);
    std::filesystem::create_directories(directory_);
    if (chmod(directory_.c_str(), 0700) != 0) {
      throw std::runtime_error("Cannot make bag directory private: " + std::string(strerror(errno)));
    }

    start_service_ = create_service<StartRecorder>("bag_recorder/start",
      [this](StartRecorder::Request::SharedPtr, StartRecorder::Response::SharedPtr) {
        requested_ = true;
        next_start_ = Clock::now();
        MaintainRecorder();
      });
    stop_service_ = create_service<StopRecorder>("bag_recorder/stop",
      [this](StopRecorder::Request::SharedPtr, StopRecorder::Response::SharedPtr) {
        requested_ = false;
        StopRecorderProcess();
      });
    led_ = create_publisher<std_msgs::msg::Int64>("/io/leds/bag", 10);
    if (buttons) {
      button_ = create_subscription<std_msgs::msg::Int64>("/io/buttons/bag", 10,
        [this](std_msgs::msg::Int64::SharedPtr msg) {
          if (msg->data == last_button_) {return;}
          last_button_ = msg->data;
          if (msg->data == 1) {
            requested_ = true;
            next_start_ = Clock::now();
            MaintainRecorder();
          } else if (msg->data == 0) {
            requested_ = false;
            StopRecorderProcess();
          }
        });
    }
    timer_ = create_wall_timer(std::chrono::seconds(period), [this]() {MaintainRecorder();});

    // Hold a close-on-exec lock for this node's lifetime. It is not inherited
    // by ros2, and prevents multiple recorder nodes sharing a data directory.
    lock_fd_ = open((directory_ / ".recorder.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock_fd_ < 0) {throw std::runtime_error("Cannot open recorder lock");}
    if (flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      close(lock_fd_);
      lock_fd_ = -1;
      throw std::runtime_error("Another recorder node already owns this bag directory");
    }
    RCLCPP_INFO(get_logger(), "Bag storage: %s; autostart: %s",
      directory_.c_str(), requested_ ? "true" : "false");
    // Start from the executor, after construction has completed successfully.
  }

  ~BagRecorderNode() override
  {
    StopRecorderProcess();
    if (lock_fd_ >= 0) {close(lock_fd_);}
  }

private:
  void PublishStatus()
  {
    if (!rclcpp::ok()) {return;}
    std_msgs::msg::Int64 status;
    status.data = pid_ > 0 ? 1 : 0;
    led_->publish(status);
  }

  bool ReapRecorder()
  {
    if (pid_ <= 0) {return true;}
    int status = 0;
    const pid_t result = waitpid(pid_, &status, WNOHANG);
    if (result == 0 || (result < 0 && errno == EINTR)) {return false;}
    if (result < 0 && errno != ECHILD) {
      RCLCPP_ERROR(get_logger(), "waitpid failed: %s", strerror(errno));
      return false;
    }
    if (result > 0) {
      if (WIFEXITED(status)) {
        RCLCPP_INFO(get_logger(), "Recorder exited with code %d", WEXITSTATUS(status));
      } else if (WIFSIGNALED(status)) {
        RCLCPP_WARN(get_logger(), "Recorder exited on signal %d", WTERMSIG(status));
      }
    }
    pid_ = -1;
    PublishStatus();
    return true;
  }

  void StopRecorderProcess()
  {
    if (ReapRecorder()) {return;}
    // Each child has its own process group. Never kill unrelated ROS nodes.
    const std::pair<int, int> stages[] = {{SIGINT, 30}, {SIGTERM, 5}, {SIGKILL, 5}};
    for (const auto & stage : stages) {
      RCLCPP_INFO(get_logger(), "Stopping recorder (signal %d)", stage.first);
      if (kill(-pid_, stage.first) != 0 && errno != ESRCH) {
        RCLCPP_ERROR(get_logger(), "Cannot signal recorder: %s", strerror(errno));
      }
      const auto deadline = Clock::now() + std::chrono::seconds(stage.second);
      do {
        if (ReapRecorder()) {return;}
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (Clock::now() < deadline);
      RCLCPP_WARN(get_logger(), "Recorder stop timed out; escalating signal");
    }
  }

  void SpawnRecorder()
  {
    const auto now = std::time(nullptr);
    std::tm utc{};
    gmtime_r(&now, &utc);
    std::random_device random;
    std::ostringstream name;
    name << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << "_" << getpid()
         << "_" << std::hex << random() << random();
    const auto output = (directory_ / name.str()).string();
    std::vector<std::string> args = {"ros2", "bag", "record", "--all", "--output", output,
      "--storage", "sqlite3", "--max-bag-size", std::to_string(max_size_),
      "--max-bag-duration", std::to_string(max_duration_)};
    std::vector<char *> argv;
    for (auto & arg : args) {argv.push_back(arg.data());}
    argv.push_back(nullptr);

    // posix_spawn avoids executing C++/ROS code after fork in a multithreaded node.
    posix_spawnattr_t attr;
    posix_spawn_file_actions_t actions;
    int result = posix_spawnattr_init(&attr);
    if (result != 0) {throw std::runtime_error(strerror(result));}
    result = posix_spawn_file_actions_init(&actions);
    if (result != 0) {
      posix_spawnattr_destroy(&attr);
      throw std::runtime_error(strerror(result));
    }
    sigset_t defaults, mask;
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGINT);
    sigaddset(&defaults, SIGTERM);
    sigemptyset(&mask);
    result = posix_spawnattr_setflags(&attr,
      POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK);
    if (result == 0) {result = posix_spawnattr_setpgroup(&attr, 0);}
    if (result == 0) {result = posix_spawnattr_setsigdefault(&attr, &defaults);}
    if (result == 0) {result = posix_spawnattr_setsigmask(&attr, &mask);}
    if (result == 0) {
      result = posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    }
    if (result == 0) {
      result = posix_spawnp(&pid_, "ros2", &actions, &attr, argv.data(), environ);
    }
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attr);
    if (result != 0) {
      pid_ = -1;
      RCLCPP_ERROR(get_logger(), "Cannot start recorder: %s; retrying", strerror(result));
      next_start_ = Clock::now() + std::chrono::seconds(10);
      return;
    }
    RCLCPP_INFO(get_logger(), "Recording all discovered topics to %s", output.c_str());
    PublishStatus();
  }

  void MaintainRecorder()
  {
    if (pid_ > 0 && ReapRecorder()) {
      next_start_ = Clock::now() + std::chrono::seconds(10);
    }
    if (!requested_) {return;}
    struct statvfs disk{};
    if (statvfs(directory_.c_str(), &disk) != 0) {
      RCLCPP_ERROR(get_logger(), "Cannot check bag filesystem: %s; stopping", strerror(errno));
      StopRecorderProcess();
      return;
    }
    const uint64_t free_kb = static_cast<uint64_t>(disk.f_bavail) * disk.f_frsize / 1024;
    const auto reserve = static_cast<uint64_t>(min_free_kb_);
    if (free_kb < reserve) {
      if (!disk_blocked_) {
        RCLCPP_WARN(get_logger(), "Low disk space; stopping recording and preserving existing bags");
      }
      disk_blocked_ = true;
      StopRecorderProcess();
      return;
    }
    if (disk_blocked_ && free_kb - reserve < static_cast<uint64_t>(resume_margin_kb_)) {return;}
    disk_blocked_ = false;
    if (pid_ <= 0 && Clock::now() >= next_start_) {
      try {
        SpawnRecorder();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Cannot prepare recorder: %s", error.what());
        next_start_ = Clock::now() + std::chrono::seconds(10);
      }
    }
    PublishStatus();
  }

  std::filesystem::path directory_;
  int64_t min_free_kb_, resume_margin_kb_, max_size_, max_duration_;
  bool requested_ = false;
  bool disk_blocked_ = false;
  pid_t pid_ = -1;
  int lock_fd_ = -1;
  int64_t last_button_ = -1;
  Clock::time_point next_start_ = Clock::now();
  rclcpp::Service<StartRecorder>::SharedPtr start_service_;
  rclcpp::Service<StopRecorder>::SharedPtr stop_service_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr button_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr led_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  umask(0077);
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<BagRecorderNode>();
    rclcpp::spin(node);
    // Destruction stops and reaps the child even after ROS shutdown.
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("bag_recorder_service"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
