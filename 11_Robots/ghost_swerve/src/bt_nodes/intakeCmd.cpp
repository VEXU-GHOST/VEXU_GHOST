#include "ghost_swerve/bt_nodes/intakeCmd.hpp"

namespace ghost_swerve {

IntakeCmd::IntakeCmd(const std::string& name, const BT::NodeConfig& config,
			std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
			std::shared_ptr<SwerveModel> swerve_ptr,
			double burnout_absolute_rpm_threshold,
			double burnout_stall_duration_ms,
			double burnout_cooldown_duration_ms,
			double lift_setpoint) :
	BT::SyncActionNode(name, config),
	rclcpp::Node("intake_cmd"),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr),
	burnout_absolute_rpm_threshold_(burnout_absolute_rpm_threshold),
	burnout_stall_duration_ms_(burnout_stall_duration_ms),
	burnout_cooldown_duration_ms_(burnout_cooldown_duration_ms),
	lift_setpoint_(lift_setpoint){
}

// It is mandatory to define this STATIC method.
BT::PortsList IntakeCmd::providedPorts(){
	return {
	    BT::InputPort<bool>("in"),
	};
}

template <typename T>
T IntakeCmd::get_input(std::string key){
	BT::Expected<T> input = getInput<T>(key);
	// Check if expected is valid. If not, throw its error
	if(!input){
		throw BT::RuntimeError("missing required input [" + key + "]: ",
		                       input.error() );
	}
	return input.value();
}

BT::NodeStatus IntakeCmd::tick() {
	auto status = BT::NodeStatus::FAILURE;

	bool in = get_input<bool>("in");

	// Intake
	double intake_voltage;
	bool intake_command;
	if(!in){
		intake_command = true;
		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
		intake_voltage = -1.0;
	}
	else if(in){
		intake_command = true;
		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 2500);
		intake_voltage = 1.0;
	}
	else{
		intake_command = false;
		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
		intake_voltage = 0.0;
	}

	bool intake_up = false;
	double intake_lift_target;
	if(!in){// intake lift
		intake_up = true;
		intake_lift_target = 0.0;
	}
	else{
		intake_up = false;
		intake_lift_target = 7.0;
	}

	if(std::fabs(rhi_ptr_->getMotorPosition("intake_lift_motor") - intake_lift_target) < 0.05){
		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_lift_motor", 0);
		if(intake_up && !intake_command){
			intake_voltage = 0.0;
		}
	}
	else{
		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_lift_motor", 2500);
		if(intake_up && !intake_command){
			rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 1000);
			intake_voltage = -1.0;
		}
	}

	rhi_ptr_->setMotorPositionCommand("intake_lift_motor", intake_lift_target);


	// If INTAKE_MOTOR stalling, update state and timer
	if((intake_command)
		&& (std::fabs(rhi_ptr_->getMotorVelocityRPM("intake_motor")) < burnout_absolute_rpm_threshold_)){
		if(!intake_stalling_){
			intake_stall_start_ = std::chrono::system_clock::now();
			
			intake_stalling_ = true;
		}
	}
	else{
		intake_stalling_ = false;
	}

	// If INTAKE_MOTOR stalled for too long, start cooldown period
	if(!intake_cooling_down_ && intake_stalling_
		&& (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - intake_stall_start_).count() > burnout_stall_duration_ms_) ){
		status = BT::NodeStatus::SUCCESS;
		intake_voltage = 0;
		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
		// intake_stalling_ = false;
		// intake_cooling_down_ = true;
		// intake_cooldown_start_ = std::chrono::system_clock::now();
	}

	// Enforce INTAKE_MOTOR cooldown period
	// if(intake_cooling_down_){
	// 	if((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - intake_cooldown_start_).count() <= burnout_cooldown_duration_ms_) && intake_command){
	// 		rhi_ptr_->setMotorCurrentLimitMilliAmps("intake_motor", 0);
	// 		rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", 0);
	// 	}
	// 	else{
	// 		intake_cooling_down_ = false;
	// 	}
	// }


	rhi_ptr_->setMotorVoltageCommandPercent("intake_motor", intake_voltage);
	RCLCPP_INFO(this->get_logger(), "intaking");

	return status;
}


} // namespace ghost_swerve
