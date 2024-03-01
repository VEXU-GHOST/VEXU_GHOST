#include "ghost_swerve/bt_nodes/moveToPose.hpp"

using std::placeholders::_1;

namespace ghost_swerve {

// If your Node has ports, you must use this constructor signature
MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
					   std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::StatefulActionNode(name, config),
	rclcpp::Node("move_to_pose_node"),
	rhi_ptr_(rhi_ptr),
	swerve_ptr_(swerve_ptr){
	command_pub_ = create_publisher<ghost_msgs::msg::DrivetrainCommand>("/motion_planner/command", 10);
	started_ = false;
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPose::providedPorts(){
	return {
	    BT::InputPort<double>("posX"),
	    BT::InputPort<double>("posY"),
	    BT::InputPort<double>("theta"),
	    BT::InputPort<double>("threshold"),
	    BT::InputPort<int>("timeout"),
	    // BT::InputPort<double>("velX"),
	    // BT::InputPort<double>("velY"),
	    // BT::InputPort<double>("omega"),
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

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPose::onStart(){
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPose::onHalted(){

}

BT::NodeStatus MoveToPose::onRunning() {
	double posX = get_input<double>("posX");
	double posY = get_input<double>("posY");
	double theta = get_input<double>("theta");
	double threshold = get_input<double>("threshold");
	int timeout = get_input<int>("timeout");
	// double velX = get_input<double>("velX");
	// double velY = get_input<double>("velY");
	// double omega = get_input<double>("omega");

	theta *= ghost_util::DEG_TO_RAD;

	swerve_ptr_->calculateKinematicSwerveControllerMoveToPoseWorld(posX, posY, theta);

	if((abs(posX - swerve_ptr_->getOdometryLocation().x()) < threshold) && (abs(posY - swerve_ptr_->getOdometryLocation().y()) < threshold)){
		return BT::NodeStatus::SUCCESS;
	}

	if(started_){
		auto now = std::chrono::system_clock::now();
		int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
		if (time_elapsed > timeout){
			RCLCPP_WARN(this->get_logger(), "MoveToPose Timeout: %i ms elapsed", time_elapsed);
			started_ = false;
			return BT::NodeStatus::FAILURE;
		}
	} else {
		start_time_ = std::chrono::system_clock::now();
		started_ = true;
	}
	 
	return BT::NodeStatus::RUNNING;

	// double w,x,y,z;
	// ghost_util::yawToQuaternionDeg(theta, w, x, y, z);
	// ghost_msgs::msg::DrivetrainCommand msg{};
	// msg.pose.pose.position.x = posX;
	// msg.pose.pose.position.y = posY;
	// msg.pose.pose.orientation.x = x;
	// msg.pose.pose.orientation.y = y;
	// msg.pose.pose.orientation.z = z;
	// msg.pose.pose.orientation.w = w;

	// // geometry_msgs::msg::TwistStamped twist{};
	// msg.twist.twist.linear.x = velX;
	// msg.twist.twist.linear.y = velY;
	// msg.twist.twist.angular.z = omega * ghost_util::DEG_TO_RAD;

	// command_pub_->publish(msg);

	// RCLCPP_INFO(this->get_logger(), "posX: %f", posX);
	// RCLCPP_INFO(this->get_logger(), "posY: %f", posY);
	// RCLCPP_INFO(this->get_logger(), "theta: %f", theta);
	// RCLCPP_INFO(this->get_logger(), "velX: %f", velX);
	// RCLCPP_INFO(this->get_logger(), "velY: %f", velY);
	// RCLCPP_INFO(this->get_logger(), "omega: %f", omega);

	// // subscrive to odom and check if reached goal
	// // also timeout

	// return BT::NodeStatus::SUCCESS;
}

} // namespace ghost_swerve
