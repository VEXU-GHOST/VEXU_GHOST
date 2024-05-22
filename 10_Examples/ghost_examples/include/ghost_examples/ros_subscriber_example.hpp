// This line is always added to the top of header files. "Including" a header file is basically like copy-pasting it
// into your code, and this line prevents it from getting pasted multiple times if we include multiple headers that also
// include each other.
#pragma once

#include <memory>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// The using declaration allows us to use items from a namespace without saying the full name everytime.
// EX. _1, instead of std::placeholders::_1 all over the code.
// NOTE: Don't put using declarations in a header file, or that short-hand gets propogated to other places...
using std::placeholders::_1;

// We typically define code inside a namespace so that it is easier to tell what it belongs to when things get complicated.
namespace ghost_examples
{

// In C++, we typically "declare" classes and functions in a header file, and then actually "define" them in the cpp file.
// Below is a class for our subscriber example. It also has a base/parent class that is a rclcpp::Node.
class ROSSubscriberExample : public rclcpp::Node
{
public:
  // Everything under "public" can be accessed from outside the class. So you can call any of these functions from your main.

  // This is the constructor of the class. Every class will have this function.
  // You can also have constructors which take arguments or multiple constructors.
  ROSSubscriberExample();

  // In ROS, we communicate with other code through "topics." Basically, we define a function that processes some data
  // and then give that function to ROS, and whenever the data shows up, the function gets applied.
  // This function is called whenever a PoseWithCovarianceStamped msg is received from ROS.
  void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // These functions are for our GTest example, and have nothing to do with the ROS Subscriber :).
  int add_ints(int a, int b);
  float add_floats(float a, float b);
  void function_that_throws_error();
  void do_nothing();

protected:
  // "Protected" is like Private (explained below), except classes which inherit from this class can still
  // access the methods and variables.

private:
  // Everything under "private" is ONLY accessible from inside the class. That way you can store data and be sure that no one
  // can change it without you knowing. Derived classes cannot access private members of a base class.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

} // namespace ghost_examples
