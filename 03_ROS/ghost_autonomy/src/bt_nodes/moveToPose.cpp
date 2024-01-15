#include "ghost_autonomy/bt_nodes/moveToPose.hpp"

using std::placeholders::_1;

// If your Node has ports, you must use this constructor signature
MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
		BT::SyncActionNode(name, config),
		rclcpp::Node("move_to_pose_node"),
		robot_hardware_interface_ptr_(robot_hardware_interface_ptr){
		pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("motionPlanning/pose", 10);
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPose::providedPorts(){
		return {
		        BT::InputPort<double>("posX"),
		        BT::InputPort<double>("posY"),
		        BT::InputPort<double>("quatX"),
		        BT::InputPort<double>("quatY"),
		        BT::InputPort<double>("quatZ"),
		        BT::InputPort<double>("quatW")
		};
}

double MoveToPose::get_input(std::string key){
		BT::Expected<double> input = getInput<double>(key);
		// Check if expected is valid. If not, throw its error
		if(!input){
				throw BT::RuntimeError("missing required input [" + key + "]: ",
				                       input.error() );
		}
		return input.value();
}

// Override the virtual function tick()
BT::NodeStatus MoveToPose::tick() {
		double posX = get_input("posX");
		double posY = get_input("posY");
		double quatX = get_input("quatX");
		double quatY = get_input("quatY");
		double quatZ = get_input("quatZ");
		double quatW = get_input("quatW");

		geometry_msgs::msg::PoseStamped msg{};
		msg.pose.position.x = posX;
		msg.pose.position.y = posY;
		msg.pose.orientation.x = quatX;
		msg.pose.orientation.y = quatY;
		msg.pose.orientation.z = quatZ;
		msg.pose.orientation.w = quatW;
		pose_pub_->publish(msg);

		RCLCPP_INFO(this->get_logger(), "posX: %f", posX);
		RCLCPP_INFO(this->get_logger(), "posY: %f", posX);
		RCLCPP_INFO(this->get_logger(), "quatX: %f", quatX);
		RCLCPP_INFO(this->get_logger(), "quatY: %f", quatY);
		RCLCPP_INFO(this->get_logger(), "quatZ: %f", quatZ);
		RCLCPP_INFO(this->get_logger(), "quatW: %f", quatW);

		return BT::NodeStatus::SUCCESS;
}