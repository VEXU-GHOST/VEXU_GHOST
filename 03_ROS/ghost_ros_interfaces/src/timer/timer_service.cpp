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
#include "ghost_msgs/srv/check_timer.hpp"
#include "ghost_msgs/srv/start_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using ghost_msgs::srv::CheckTimer;
using ghost_msgs::srv::StartTimer;
using std::placeholders::_1;
using std::placeholders::_2;

class TimerService : public rclcpp::Node
{
public:
  TimerService()
  : rclcpp::Node("timer_service")
  {
    declare_parameter<int>("max_timers", 10);
    max_timers_ = get_parameter("max_timers").as_int();

    this->start_timer_service_ = this->create_service<StartTimer>(
      "start_timer",
      std::bind(&TimerService::StartTimerCallback, this, _1, _2));
    this->check_timer_service_ = this->create_service<CheckTimer>(
      "check_timer",
      std::bind(&TimerService::CheckTimerCallback, this, _1, _2));
  }

  void StartTimerCallback(
    const std::shared_ptr<StartTimer::Request> req,
    std::shared_ptr<StartTimer::Response> res);
  bool ClearExpiredTimers();
  void CheckTimerCallback(
    const std::shared_ptr<CheckTimer::Request> req,
    std::shared_ptr<CheckTimer::Response> res);

private:
  rclcpp::Service<StartTimer>::SharedPtr start_timer_service_;
  rclcpp::Service<CheckTimer>::SharedPtr check_timer_service_;
  unsigned int max_timers_;
  std::unordered_map<std::string, std::pair<rclcpp::Time, uint64_t>> timers_;
};

void TimerService::StartTimerCallback(
  const std::shared_ptr<StartTimer::Request> req,
  std::shared_ptr<StartTimer::Response> res)
{
  ClearExpiredTimers();

  if ((timers_.size() >= max_timers_)) {
    res->success = false;
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      std::string(
        "Unable to create timer with name \"" + req->timer_name + "\".\n" +
        "\tReason: Exceeds maximum timer limit.").c_str());
    return;
  }

  if (timers_.find(req->timer_name) != timers_.end()) {
    res->success = false;
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      std::string(
        "Unable to create timer with name \"" + req->timer_name + "\".\n" +
        "\tReason: Duplicate timer name.").c_str());
    return;
  }

  timers_[req->timer_name].first = this->get_clock()->now();
  timers_[req->timer_name].second = req->duration_ns;

  res->success = true;
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    std::string("Created new timer with name \"" + req->timer_name + "\".").c_str());
}

bool TimerService::ClearExpiredTimers()
{
  bool madeSpace = false;

  std::vector<std::string> to_remove{};

  for (auto iter = timers_.begin(); iter != timers_.end(); iter++) {
    rclcpp::Time timer_start = iter->second.first;
    uint64_t timer_duration_ns = iter->second.second;

    if ((now() - timer_start).nanoseconds() > timer_duration_ns) {
      madeSpace = true;
      to_remove.push_back(iter->first);
    }
  }

  for (auto key : to_remove) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("rclcpp"),
      std::string("Deleted timer \"" + key + "\".").c_str());
    timers_.erase(key);
  }

  return madeSpace;
}

void TimerService::CheckTimerCallback(
  const std::shared_ptr<CheckTimer::Request> req,
  std::shared_ptr<CheckTimer::Response> res)
{
  res->valid = (timers_.find(req->timer_name) != timers_.end());

  if (!res->valid) {
    return;
  }

  res->elapsed_ns = (this->get_clock()->now() - timers_[req->timer_name].first).nanoseconds();
  res->expired = (res->elapsed_ns > timers_[req->timer_name].second);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<TimerService> node = std::make_shared<TimerService>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
