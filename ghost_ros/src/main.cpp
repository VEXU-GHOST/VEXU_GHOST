
#include "main.hpp"

SimulatedRobotNode::SimulatedRobotNode(std::string node_name): Node(node_name){
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&SimulatedRobotNode::timer_callback, this));
}

SimulatedRobotNode::~SimulatedRobotNode(){
}

void SimulatedRobotNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    
}

void SimulatedRobotNode::timer_callback(){
    auto msg = std_msgs::msg::String();
    msg.data = "Test";
    publisher_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulatedRobotNode>("ghost1");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}