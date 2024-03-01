#include "ghost_swerve/bt_nodes/swipeTail.hpp"

namespace ghost_swerve {

SwipeTail::SwipeTail(const std::string& name, const BT::NodeConfig& config,
			std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
			std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::StatefulActionNode(name, config),
	rclcpp::Node("swipe_tail"),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr){
	started_ = false;
}

// It is mandatory to define this STATIC method.
BT::PortsList SwipeTail::providedPorts(){
	return {
	    BT::InputPort<int>("num_swipes"),
	};
}

template <typename T>
T SwipeTail::get_input(std::string key){
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
BT::NodeStatus SwipeTail::onStart(){
	started_ = false;
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void SwipeTail::onHalted(){
	resetStatus();
}

BT::NodeStatus SwipeTail::onRunning() {
	auto m_digital_io = std::vector<bool>(8, false);
	auto m_digital_io_name_map = std::unordered_map<std::string, size_t>{
		{"claw", 0},
		{"right_wing", 1},
		{"left_wing", 2},
		{"tail", 3}
	};

	auto status = BT::NodeStatus::RUNNING;

	int num_swipes = get_input<int>("num_swipes");

	if(num_swipes <= 0){
		RCLCPP_ERROR(this->get_logger(), "NumSwipes: invalid input");
		return BT::NodeStatus::FAILURE;
	}

	int time_elapsed = 0;
	if(started_){
		auto now = std::chrono::system_clock::now();
		time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
	} else {
		start_time_ = std::chrono::system_clock::now();
		started_ = true;
		m_digital_io[m_digital_io_name_map.at("tail")] = true;
		rhi_ptr_->setMotorCurrentLimitMilliAmps("tail_motor", 2500);
		rhi_ptr_->setMotorPositionCommand("tail_motor", swerve_ptr_->getConfig().stick_upright_angle);
	}

	double tail_mtr_pos = rhi_ptr_->getMotorPosition("tail_motor");
	double stick_turn_offset = swerve_ptr_->getConfig().stick_turn_offset;
	#define MTR_CLOSE_TO(x) (fabs(tail_mtr_pos - x) < stick_turn_offset)

	RCLCPP_INFO(this->get_logger(), "time elapsed: %f", time_elapsed * 0.001);

	if(500 < time_elapsed && time_elapsed < 1000 * num_swipes + 500){
		if(time_elapsed % 1000 <= 400){
			rhi_ptr_->setMotorPositionCommand("tail_motor", swerve_ptr_->getConfig().stick_angle_normal);
		}
		else{
			rhi_ptr_->setMotorPositionCommand("tail_motor", swerve_ptr_->getConfig().stick_upright_angle);
		}
		status = BT::NodeStatus::RUNNING;
	} else if(time_elapsed > 1000 * num_swipes + 500){
		rhi_ptr_->setMotorPositionCommand("tail_motor", swerve_ptr_->getConfig().stick_upright_angle);
		if(MTR_CLOSE_TO(swerve_ptr_->getConfig().stick_upright_angle)){ // within n degrees of upright
			m_digital_io[m_digital_io_name_map.at("tail")] = false;
			rhi_ptr_->setMotorCurrentLimitMilliAmps("tail_motor", 100); // i'm going to give it less but not none so it can hold itself centered
		}
		status = BT::NodeStatus::SUCCESS;
	}

	rhi_ptr_->setDigitalIO(m_digital_io);
	return status;
}


} // namespace ghost_swerve
