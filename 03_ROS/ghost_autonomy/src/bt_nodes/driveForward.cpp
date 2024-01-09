#include "ghost_autonomy/bt_nodes/driveForward.hpp"

using std::placeholders::_1;

// If your Node has ports, you must use this constructor signature
DriveForward::DriveForward(const std::string& name, const BT::NodeConfig& config) :
		BT::SyncActionNode(name, config),
		rclcpp::Node("driveforward_node"){
}

// It is mandatory to define this STATIC method.
static BT::PortsList DriveForward::providedPorts(){
		return {
		        BT::InputPort<double>("distance"),
		        BT::InputPort<std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> >("robot_hardware_interface_ptr")
		};
}

// Override the virtual function tick()
BT::NodeStatus DriveForward::tick() override {
		BT::Expected<double> dist = getInput<double>("distance");
		// Check if expected is valid. If not, throw its error
		if(!dist){
				throw BT::RuntimeError("missing required input [distance]: ",
				                       dist.error() );
		}

		BT::Expected<std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> > robot_hardware_interface_ptr =
				getInput<std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> >("robot_hardware_interface_ptr");
		// Check if expected is valid. If not, throw its error
		if(!robot_hardware_interface_ptr){
				throw BT::RuntimeError("missing required input [robot_ptr]: ",
				                       robot_hardware_interface_ptr.error() );
		}

		// use the method value() to extract the valid message.
		RCLCPP_INFO(this->get_logger(), "forward dist: %f", dist.value());


		robot_hardware_interface_ptr.value()->setAutonomousStatus(true);

		RCLCPP_INFO(this->get_logger(), "auton activated: %i", robot_hardware_interface_ptr.value()->isAutonomous());

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return BT::NodeStatus::SUCCESS;
}