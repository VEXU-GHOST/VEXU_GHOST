#include "ghost_swerve/bt_nodes/climb.hpp"

namespace ghost_swerve {

Climb::Climb(const std::string& name, const BT::NodeConfig& config,
			std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
			std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::StatefulActionNode(name, config),
	rclcpp::Node("climb"),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr){
}

// It is mandatory to define this STATIC method.
BT::PortsList Climb::providedPorts(){
	return {
	};
}

template <typename T>
T Climb::get_input(std::string key){
	BT::Expected<T> input = getInput<T>(key);
	// Check if expected is valid. If not, throw its error
	if(!input){
		throw BT::RuntimeError("missing required input [" + key + "]: ",
		                       input.error() );
	}
	return input.value();
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus Climb::onStart(){
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void Climb::onHalted(){
	resetStatus();
}

BT::NodeStatus Climb::onRunning() {
	auto m_digital_io = std::vector<bool>(8, false);
	auto m_digital_io_name_map = std::unordered_map<std::string, size_t>{
		{"claw", 0},
		{"right_wing", 1},
		{"left_wing", 2},
		{"tail", 3}
	};

	auto status = BT::NodeStatus::RUNNING;

	RCLCPP_INFO(this->get_logger(), "Climbing");
	
	// rhi_ptr_->setMotorPositionCommand("tail_motor", swerve_ptr_->getConfig().stick_angle_normal);

	// rhi_ptr_->setDigitalIO(m_digital_io);

	// do not return any status but RUNNING 
	// so it keeps the state at the end of the auton

	return status;
}


} // namespace ghost_swerve
