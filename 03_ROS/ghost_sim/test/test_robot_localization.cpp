/*
 * Filename: test_publisher_v5_actuator_cmd
 * Created Date: Sunday August 7th 2022
 * Author: Melissa Cruz
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Saturday September 10th 2022 10:50:34 am
 * Modified By: Melissa Cruz
 */

#include "ghost_sim/test_robot_localization.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ghost_sim {

testRobotLocalization::testRobotLocalization() :
	Node("test_robot_localization"){
	rclcpp::TimerBase::SharedPtr timer_;

	// Subscribe to Gazebo model state topic
	gz_model_state_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
		"/model_states",
		10,
		std::bind(&testRobotLocalization::modelStatesCallback, this, _1)
		);

	// Initialize Publisher
	output_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("gz/model_pose", 10);

	rclcpp::Parameter use_sim_time_param("use_sim_time", false);
	this->set_parameter(use_sim_time_param);
}


void testRobotLocalization::modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
	for(int i = 0; i < msg->name.size(); i++){
		if(msg->name[i] == "ghost1"){
			gz_model_state_input_ = msg->pose[i];
		}
	}
	output_pub_->publish(gz_model_state_input_);
}

}  // namespace ghost_sim

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_sim::testRobotLocalization>());
	rclcpp::shutdown();
	return 0;
}