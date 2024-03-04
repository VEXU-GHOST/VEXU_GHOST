#include "ghost_swerve/bt_nodes/setWing.hpp"

namespace ghost_swerve {

SetWing::SetWing(const std::string& name, const BT::NodeConfig& config,
			std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
			std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::SyncActionNode(name, config),
	rclcpp::Node("set_wing"),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr){
}

// It is mandatory to define this STATIC method.
BT::PortsList SetWing::providedPorts(){
	return {
	    BT::InputPort<bool>("state"),
	};
}

template <typename T>
T SetWing::get_input(std::string key){
	BT::Expected<T> input = getInput<T>(key);
	// Check if expected is valid. If not, throw its error
	if(!input){
		throw BT::RuntimeError("missing required input [" + key + "]: ",
		                       input.error() );
	}
	return input.value();
}

BT::NodeStatus SetWing::tick() {
	auto m_digital_io = std::vector<bool>(8, false);
	auto m_digital_io_name_map = std::unordered_map<std::string, size_t>{
		{"claw", 0},
		{"right_wing", 1},
		{"left_wing", 2},
		{"tail", 3}
	};

	auto status = BT::NodeStatus::SUCCESS;

	bool state = get_input<bool>("state");

	m_digital_io[m_digital_io_name_map.at("right_wing")] = state;
	// m_digital_io[m_digital_io_name_map.at("left_wing")] = !m_climb_mode && joy_data->btn_l2;

	rhi_ptr_->setDigitalIO(m_digital_io);
	return status;
}


} // namespace ghost_swerve
