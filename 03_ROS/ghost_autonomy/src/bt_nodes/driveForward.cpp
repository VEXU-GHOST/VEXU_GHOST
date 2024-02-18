#include "ghost_autonomy/bt_nodes/driveForward.hpp"

using std::placeholders::_1;

// If your Node has ports, you must use this constructor signature
DriveForward::DriveForward(const std::string& name, const BT::NodeConfig& config,
                           std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> robot_hardware_interface_ptr) :
		BT::SyncActionNode(name, config),
		rclcpp::Node("driveforward_node"),
		robot_hardware_interface_ptr_(robot_hardware_interface_ptr){
}

// It is mandatory to define this STATIC method.
BT::PortsList DriveForward::providedPorts(){
		return {
		        BT::InputPort<double>("distance")
		};
}

// Override the virtual function tick()
BT::NodeStatus DriveForward::tick() {
		BT::Expected<double> dist = getInput<double>("distance");
		// Check if expected is valid. If not, throw its error
		if(!dist){
				throw BT::RuntimeError("missing required input [distance]: ",
				                       dist.error() );
		}

		// use the method value() to extract the valid message.
		RCLCPP_INFO(this->get_logger(), "forward dist: %f", dist.value());


		robot_hardware_interface_ptr_->setAutonomousStatus(true);

		RCLCPP_INFO(this->get_logger(), "auton activated: %i", robot_hardware_interface_ptr_->isAutonomous());

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return BT::NodeStatus::SUCCESS;
}