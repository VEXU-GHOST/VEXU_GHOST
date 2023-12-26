#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

namespace Math_Space {

class Mather : public rclcpp::Node {
public:
	Mather(int a, int b);

	int decode(int encoded);

	void topic_callback(const std_msgs::msg::Int32::SharedPtr msg);

	void timer_callback();

private:
	int a;
	int b;
	int decodeInt;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher <std_msgs::msg::Int32>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
	size_t count_;
};

}