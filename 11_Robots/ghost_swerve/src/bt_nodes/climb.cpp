#include "ghost_swerve/bt_nodes/climb.hpp"

namespace ghost_swerve {

Climb::Climb(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node_ptr,
             std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
             std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::StatefulActionNode(name, config),
	node_ptr_(node_ptr),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr){
}

// It is mandatory to define this STATIC method.
BT::PortsList Climb::providedPorts(){
	return {
	    BT::InputPort<bool>("climbed"),
	};
}

template <typename T>
T Climb::get_input(std::string key){
	BT::Expected<T> input = getInput<T>(key);
	// Check if expected is valid. If not, throw its error
	if(!input){
		throw BT::RuntimeError("missing required input [" + key + "]: ",
		                       input.error());
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
	rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 0);
	rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 0);

	resetStatus();
}

float Climb::tempPID(std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_, const std::string &motor1, const std::string &motor2, float pos_want, double kP){
	float pos1 = rhi_ptr_->getMotorPosition(motor1);
	float pos2 = rhi_ptr_->getMotorPosition(motor2);
	float pos = (pos1 + pos2) / 2;
	float action = std::clamp((pos_want - pos) * kP, -100., 100.); // TODO ???
	if(fabs(action) < 1.5){
		action = 0;
	}
	rhi_ptr_->setMotorVoltageCommandPercent(motor1, action);
	rhi_ptr_->setMotorVoltageCommandPercent(motor2, action);
	// std::cout << "pos1: " << pos1 << " pos2: " << pos2 << " want: " << pos_want << " kP " << kP << " error " << (pos_want - pos) << " action " << action << std::endl;
	return pos - pos_want;
}

BT::NodeStatus Climb::onRunning(){
	auto m_digital_io = std::vector<bool>(8, false);
	auto m_digital_io_name_map = std::unordered_map<std::string, size_t>{
		{"claw", 0},
		{"right_wing", 1},
		{"left_wing", 2},
		{"tail", 3}
	};
	bool claw_open;
	auto status = BT::NodeStatus::RUNNING;
	bool climbed = get_input<bool>("climbed");

	if(climbed){
		lift_target = swerve_ptr_->getConfig().lift_climbed_angle;
		claw_open = false;
	}
	else{
		lift_target = swerve_ptr_->getConfig().lift_up_angle;
		claw_open = true;
	}

	RCLCPP_INFO(node_ptr_->get_logger(), "Climbing");

	// Toggle Climb Mode
	rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_right", 2500);
	rhi_ptr_->setMotorCurrentLimitMilliAmps("lift_left", 2500);

	float err = tempPID(rhi_ptr_, "lift_right", "lift_left", lift_target, swerve_ptr_->getConfig().lift_kP); // go to 90deg
	// ignore err

	m_digital_io[m_digital_io_name_map.at("claw")] = claw_open;

	rhi_ptr_->setDigitalIO(m_digital_io);

	// do not return any status but RUNNING
	// so it keeps the state at the end of the auton

	return status;
}

} // namespace ghost_swerve