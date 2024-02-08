#include "ghost_autonomy/bt_nodes/moveToPose.hpp"

using std::placeholders::_1;

// If your Node has ports, you must use this constructor signature
MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
		BT::SyncActionNode(name, config),
		rclcpp::Node("move_to_pose_node"),
		robot_hardware_interface_ptr_(robot_hardware_interface_ptr){
		command_pub_ = create_publisher<ghost_msgs::msg::DrivetrainCommand>("motionPlanning/command", 10);
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPose::providedPorts(){
		return {
		        BT::InputPort<double>("posX"),
		        BT::InputPort<double>("posY"),
		        BT::InputPort<double>("angle"),
		        BT::InputPort<double>("velX"),
		        BT::InputPort<double>("velY"),
		        BT::InputPort<double>("omega"),
		};
}
template <typename T>
T MoveToPose::get_input(std::string key){
		BT::Expected<T> input = getInput<T>(key);
		// Check if expected is valid. If not, throw its error
		if(!input){
				throw BT::RuntimeError("missing required input [" + key + "]: ",
				                       input.error() );
		}
		return input.value();
}

// Override the virtual function tick()
BT::NodeStatus MoveToPose::tick() {
		double posX = get_input<double>("posX");
		double posY = get_input<double>("posY");
		double angle = get_input<double>("angle");
		double velX = get_input<double>("velX");
		double velY = get_input<double>("velY");
		double omega = get_input<double>("omega");

		tf2::Quaternion quat;
		quat.setEuler(angle, 0, 0);

		ghost_msgs::msg::DrivetrainCommand msg{};
		msg.pose.pose.position.x = posX;
		msg.pose.pose.position.y = posY;
		msg.pose.pose.orientation.x = quat.getX();
		msg.pose.pose.orientation.y = quat.getY();
		msg.pose.pose.orientation.z = quat.getZ();
		msg.pose.pose.orientation.w = quat.getW();

		geometry_msgs::msg::TwistStamped twist{};
		msg.twist.twist.linear.x = velX;
		msg.twist.twist.linear.y = velY;
		msg.twist.twist.angular.x = omega;

		command_pub_->publish(msg);

		RCLCPP_INFO(this->get_logger(), "posX: %f", posX);
		RCLCPP_INFO(this->get_logger(), "posY: %f", posX);
		RCLCPP_INFO(this->get_logger(), "angle: %f", angle);
		RCLCPP_INFO(this->get_logger(), "velX: %f", velX);
		RCLCPP_INFO(this->get_logger(), "velY: %f", velY);
		RCLCPP_INFO(this->get_logger(), "omega: %f", omega);

		return BT::NodeStatus::SUCCESS;
}