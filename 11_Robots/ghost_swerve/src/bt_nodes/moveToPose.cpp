#include "ghost_swerve/bt_nodes/moveToPose.hpp"

using std::placeholders::_1;

namespace ghost_swerve {

// If your Node has ports, you must use this constructor signature
MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node_ptr,
                       std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr,
					   std::shared_ptr<SwerveModel> swerve_ptr) :
	BT::StatefulActionNode(name, config),
	rhi_ptr_(rhi_ptr),
	node_ptr_(node_ptr),
	swerve_ptr_(swerve_ptr){
	command_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::DrivetrainCommand>(
		"/motion_planner/command", 
		10);
	started_ = false;
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPose::providedPorts(){
	return {
	    BT::InputPort<double>("posX"),
	    BT::InputPort<double>("posY"),
	    BT::InputPort<double>("theta"),
	    BT::InputPort<double>("velX"),
	    BT::InputPort<double>("velY"),
	    BT::InputPort<double>("omega"),
	    BT::InputPort<double>("threshold"),
	    BT::InputPort<double>("angle_threshold"),
	    BT::InputPort<double>("speed"),
	    BT::InputPort<int>("timeout"),
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
	started_ = false;
	// plan_time_ = std::chrono::();
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPose::onHalted(){
	resetStatus();
}

BT::NodeStatus MoveToPose::onRunning() {
	double posX = get_input<double>("posX");
	double posY = get_input<double>("posY");
	double theta = get_input<double>("theta");
	double velX = get_input<double>("velX");
	double velY = get_input<double>("velY");
	double omega = get_input<double>("omega");
	double threshold = get_input<double>("threshold");
	double angle_threshold = get_input<double>("angle_threshold");
	double speed = get_input<double>("speed");
	int timeout = get_input<int>("timeout");

	double tile_to_meters = 0.6096;
	posX *= tile_to_meters;
	posY *= tile_to_meters;

	theta *= ghost_util::DEG_TO_RAD;
	angle_threshold *= ghost_util::DEG_TO_RAD;

	// swerve_ptr_->calculateKinematicSwerveControllerMoveToPoseWorld(posX, posY, theta);

	double w,x,y,z;
	ghost_util::yawToQuaternionRad(theta, w, x, y, z);
	ghost_msgs::msg::DrivetrainCommand msg{};
	msg.pose.pose.position.x = posX;
	msg.pose.pose.position.y = posY;
	msg.pose.pose.orientation.x = x;
	msg.pose.pose.orientation.y = y;
	msg.pose.pose.orientation.z = z;
	msg.pose.pose.orientation.w = w;
	msg.pose.pose.position.z = threshold;
	msg.twist.twist.angular.x = angle_threshold;
	msg.speed = speed;

	// geometry_msgs::msg::TwistStamped twist{};
	msg.twist.twist.linear.x = velX;
	msg.twist.twist.linear.y = velY;
	msg.twist.twist.angular.z = omega * ghost_util::DEG_TO_RAD;

	if( (abs(posX - swerve_ptr_->getWorldLocation().x()) < threshold) &&
		(abs(posY - swerve_ptr_->getWorldLocation().y()) < threshold) &&
		(abs(ghost_util::SmallestAngleDistRad(theta, swerve_ptr_->getWorldAngleRad())) < angle_threshold)){
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: Success");
		return BT::NodeStatus::SUCCESS;
	}

	if(started_){
		auto now = std::chrono::system_clock::now();
		int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
		// int time_elapsed_since_plan = std::chrono::duration_cast<std::chrono::milliseconds>(now - plan_time_).count();
		// RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: %i ms elapsed", time_elapsed);
		if (timeout > 0){
			if (time_elapsed > timeout){
				RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPose Timeout: %i ms elapsed", time_elapsed);
				// started_ = false;
				// return BT::NodeStatus::FAILURE;
			// } else if (time_elapsed_since_plan > 10000){
				start_time_ = std::chrono::system_clock::now();
				command_pub_->publish(msg);
				RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: sent command");
				// plan_time_ = std::chrono::system_clock::now();
			}
		} else {
			if (time_elapsed > abs(timeout)){
				return BT::NodeStatus::SUCCESS;
			}
		}
	} else {
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPose: Started");
		start_time_ = std::chrono::system_clock::now();
		started_ = true;
		command_pub_->publish(msg);
		
		RCLCPP_INFO(node_ptr_->get_logger(), "posX: %f", posX);
		RCLCPP_INFO(node_ptr_->get_logger(), "posY: %f", posY);
		RCLCPP_INFO(node_ptr_->get_logger(), "theta: %f", theta);
		RCLCPP_INFO(node_ptr_->get_logger(), "velX: %f", velX);
		RCLCPP_INFO(node_ptr_->get_logger(), "velY: %f", velY);
		RCLCPP_INFO(node_ptr_->get_logger(), "omega: %f", omega);
	}
	 
	return BT::NodeStatus::RUNNING;
}

} // namespace ghost_swerve
