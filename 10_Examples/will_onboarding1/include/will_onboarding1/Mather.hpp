#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace math_space {

/*
        Class defining the Mather class:

        The Mather class is a ROS node that can take in an input and output a corresponding value depending on the initial arguments the Mather
        object is constructed with and which operation you want done.

        inputs - operations:

                1 - addion
                2 - multiplication
                3 - subtraction
                4 - modulus
                other - will simply output a 0

 */
class Mather : public rclcpp::Node {
public:

	/*
	        Mather Constructor, the parameters are the two argumets of the operation you want to perform
	 */
	Mather(int m_first, int m_second);

	/*
	        Parameter is the input value(read from a member variable that gets automatically updated in the subscriber callback function) and output
	        is the result of the operation
	 */
	int decode(int encoded);

	/*
	        This is the subscriber callback funtion, this function subscribes to the topic "/input_topic" and updates the member variable m_decode_int
	 */
	void topic_callback(const std_msgs::msg::Int32::SharedPtr msg);

	/*
	        This is the publisher callback funtion, this function outputs the result of the function "decode" to the topic "/output_topic"
	 */
	void timer_callback();

private:
	int m_first;// this is the first element of the operation
	int m_second;// this is the second element of the operation
	int m_decode_int;// this is the member variable that stores the output for the function "decode"

	rclcpp::TimerBase::SharedPtr timer_; // this is a shared pointer to the ROS timer object
	rclcpp::Publisher <std_msgs::msg::Int32>::SharedPtr publisher_; // this is "Mather"'s publisher
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;// this is "Mather"'s subscriber
	size_t count_;// I honestly don't know what this does.
};

}