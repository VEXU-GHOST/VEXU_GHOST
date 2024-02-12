#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include "ghost_motion_planner_core/motion_planner.hpp"

using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::RobotHardwareInterface;
using ghost_v5_interfaces::util::loadRobotConfigFromYAMLFile;
using std::placeholders::_1;

namespace ghost_motion_planner {

void MotionPlanner::configure(){
	// std::cout << "Configuring Motion Planner" << std::endl;
	// node_ptr_ = std::make_shared<rclcpp::Node>("motion_planner_node");

	loadRobotHardwareInterface();

	sensor_update_sub_ = create_subscription<ghost_msgs::msg::V5SensorUpdate>(
		"/v5/sensor_update",
		10,
		std::bind(&MotionPlanner::sensorUpdateCallback, this, _1)
		);

	pose_command_sub_ = create_subscription<ghost_msgs::msg::DrivetrainCommand>(
		"/motion_planner/command",
		10,
		std::bind(&MotionPlanner::setNewCommand, this, _1)
		);

	trajectory_pub_ = create_publisher<ghost_msgs::msg::RobotTrajectory>(
		"/motion_planner/trajectory",
		10);

	// start_time_ = std::chrono::system_clock::now();

	initialize();
	// configured_ = true;
}

void MotionPlanner::loadRobotHardwareInterface(){
	// Get YAML path from ROS Param
	declare_parameter("robot_config_yaml_path", "");
	std::string robot_config_yaml_path = get_parameter("robot_config_yaml_path").as_string();

	// Load RobotHardwareInterface from YAML
	auto device_config_map = loadRobotConfigFromYAMLFile(robot_config_yaml_path);
	robot_hardware_interface_ptr_ = std::make_shared<RobotHardwareInterface>(device_config_map, hardware_type_e::COPROCESSOR);
}

void MotionPlanner::sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
	if(!planning_){
		fromROSMsg(*robot_hardware_interface_ptr_, *msg);
		// make sure it doesnt overwrite values during trajectory calculation
	}
	// update values for trajectory calculation
}

void MotionPlanner::setNewCommand(const ghost_msgs::msg::DrivetrainCommand::SharedPtr cmd){
	planning_ = true;
	generateMotionPlan(cmd);
	planning_ = false;
}

} // namespace ghost_motion_planner