#include <chrono>
#include <memory>
#include <string>

#include <std_msgs/msg/int32.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

class Mather : public rclcpp::Node {
public:
	Mather() :
		Node("math_node"),
		count_(0),
		a(1),
		b(1){
		// publisher stuff
		publisher_ = this->create_publisher<std_msgs::msg::Int32>("math/topic", 10);
		timer_ = this->create_wall_timer(
			500ms, std::bind(&Mather::timer_callback, this));

		// subscriber stuff
		subscription_ = this->create_subscription<std_msgs::msg::Int32>(
			"math/topic", 10, std::bind(&Mather::topic_callback, this, _1));
	}

	int add(){
		return a + b;
	}

	int mult(){
		return a * b;
	}

private:
	int a;
	int b;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher <std_msgs::msg::Int32>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
	size_t count_;

	void timer_callback(){
		std_msgs::msg::Int32 msg;
		msg.data = a + b;
		publisher_->publish(msg);
	}

	void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
		this->a = msg->data;
		RCLCPP_INFO(this->get_logger(), "the current value is: %d\n",(a + b));
	}
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Mather>());
	rclcpp::shutdown();
	return 0;
}