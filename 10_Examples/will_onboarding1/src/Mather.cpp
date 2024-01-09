#include "will_onboarding1/Mather.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

namespace math_space {

Mather::Mather(int a, int b) :
	Node("math_node"),
	count_(0),
	m_first(a),
	m_second(b){
	// publisher stuff
	publisher_ = this->create_publisher<std_msgs::msg::Int32>("/output_topic", 10);
	timer_ = this->create_wall_timer(
		500ms, std::bind(&Mather::timer_callback, this));

	// subscriber stuff
	subscription_ = this->create_subscription<std_msgs::msg::Int32>(
		"/input_topic", 10, std::bind(&Mather::topic_callback, this, _1));
}

int Mather::decode(int encoded){
	if(encoded == 1){
		return m_first + m_second;
	}
	else if(encoded == 2){
		return m_first * m_second;
	}
	else if(encoded == 3){
		return m_first - m_second;
	}
	else if(encoded == 4){
		return m_first % m_second;
	}
	else{
		return 0;
	}
}

// publisher callback
void Mather::timer_callback(){
	std_msgs::msg::Int32 msg;
	msg.data = decode(m_decode_int);
	RCLCPP_INFO(this->get_logger(), "the current value is: %d\n",(decode(m_decode_int)));
	publisher_->publish(msg);
}

// subscriber callback
void Mather::topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
	this->m_decode_int = msg->data;
}

}