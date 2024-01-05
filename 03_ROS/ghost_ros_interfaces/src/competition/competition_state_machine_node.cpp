#include <pluginlib/class_loader.hpp>
#include "ghost_ros_interfaces/competition/v5_robot_base.hpp"

#include <iostream>

using ghost_ros_interfaces::V5RobotBase;

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);

	// Pass name of plugin which is derived from V5RobotBase (e.g. my_robot_pkg::MyRobotPlugin)
	std::string plugin_name = std::string(argv[1]);

	// Pass robot name for ROS Node Name (e.g. "ghost_swerve" results in node named "ghost_swerve_node")
	std::string robot_name = std::string(argv[2]);

	pluginlib::ClassLoader<V5RobotBase> robot_class_loader("ghost_ros_interfaces", "ghost_ros_interfaces::V5RobotBase");
	std::shared_ptr<V5RobotBase> v5_robot_base_ptr;
	try{
		std::cout << "Loading robot plugin of type: " << plugin_name << std::endl;
		v5_robot_base_ptr = robot_class_loader.createSharedInstance(plugin_name);
	}
	catch(pluginlib::PluginlibException& e){
		std::cout << "The plugin failed to load for some reason. Error: " << std::endl << e.what() << std::endl;
	}

	try{
		v5_robot_base_ptr->configure(robot_name);
	}
	catch(const std::exception& e){
		std::cout << "Failed to configure plugin for some reason. Error: " << std::endl << e.what() << std::endl;
	}

	rclcpp::spin(v5_robot_base_ptr->getROSNodePtr());
	rclcpp::shutdown();
	return 0;
}