#include <pluginlib/class_loader.hpp>
#include "ghost_competition_ros/v5_robot_base.hpp"

using ghost_competition_ros::V5RobotBase;

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);

	pluginlib::ClassLoader<V5RobotBase> robot_class_loader("ghost_competition_ros", "ghost_competition_ros::V5RobotBase");
	std::shared_ptr<V5RobotBase> v5_robot_base_ptr;
	try{
		v5_robot_base_ptr = robot_class_loader.createSharedInstance("polygon_plugins::Triangle");
	}
	catch(pluginlib::PluginlibException& ex){
		std::cout << "The plugin failed to load for some reason. Error: " << std::endl << ex.what() << std::endl;
	}

	rclcpp::spin(v5_robot_base_ptr->getROSNodePtr());
	rclcpp::shutdown();
	return 0;
}