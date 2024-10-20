#include "ghost_tank/bt_nodes/moveToPoseTrapezoid.hpp"

using std::placeholders::_1;

namespace ghost_tank {

// If your Node has ports, you must use this constructor signature
MoveToPoseTrapezoid::MoveToPoseTrapezoid(const std::string& name, const BT::NodeConfig& config) :
	BT::StatefulActionNode(name, config){
    blackboard_ = config.blackboard;
	blackboard_->get("tank_model_ptr", tank_ptr_);
	blackboard_->get("node_ptr", node_ptr_);

	node_ptr_->declare_parameter("behavior_tree.trapezoid_planner_topic", "/motion_planner/trapezoid_command");
	std::string trapezoid_planner_topic = node_ptr_->get_parameter("behavior_tree.trapezoid_planner_topic").as_string();

	command_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::DrivetrainCommand>(
		trapezoid_planner_topic,
		10);
	started_ = false;
}

// It is mandatory to define this STATIC method.
BT::PortsList MoveToPoseTrapezoid::providedPorts(){
	return {
	    BT::InputPort<double>("posX"),
	    BT::InputPort<double>("posY"),
	    BT::InputPort<double>("theta"),
	    BT::InputPort<double>("velX"),
	    BT::InputPort<double>("velY"),
	    BT::InputPort<double>("omega"),
	    BT::InputPort<double>("threshold"),
	    BT::InputPort<double>("angle_threshold"),
	    BT::InputPort<double>("max_speed"),
	    BT::InputPort<int>("timeout"),
	};
}

/// Method called once, when transitioning from the state IDLE.
/// If it returns RUNNING, this becomes an asynchronous node.
BT::NodeStatus MoveToPoseTrapezoid::onStart(){
	started_ = false;
	// plan_time_ = std::chrono::();
	return BT::NodeStatus::RUNNING;
}

/// when the method halt() is called and the action is RUNNING, this method is invoked.
/// This is a convenient place todo a cleanup, if needed.
void MoveToPoseTrapezoid::onHalted(){
	resetStatus();
}

BT::NodeStatus MoveToPoseTrapezoid::onRunning() {
	double posX = BT_Util::get_input<double>(this, "posX");
	double posY = BT_Util::get_input<double>(this, "posY");
	double theta = BT_Util::get_input<double>(this, "theta");
	double velX = BT_Util::get_input<double>(this, "velX");
	double velY = BT_Util::get_input<double>(this, "velY");
	double omega = BT_Util::get_input<double>(this, "omega");
	double threshold = BT_Util::get_input<double>(this, "threshold");
	double angle_threshold = BT_Util::get_input<double>(this, "angle_threshold");
	double max_speed = BT_Util::get_input<double>(this, "max_speed");
	int timeout = BT_Util::get_input<int>(this, "timeout");

	// tile conversion
	double tile_to_meters = 0.6096;
	posX *= tile_to_meters;
	posY *= tile_to_meters;

	// convert angle values to radians before sending
	theta *= ghost_util::DEG_TO_RAD;
	angle_threshold *= ghost_util::DEG_TO_RAD;

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
	msg.speed = max_speed;

	msg.twist.twist.linear.x = velX;
	msg.twist.twist.linear.y = velY;
	msg.twist.twist.angular.z = omega * ghost_util::DEG_TO_RAD;

	if( (abs(posX - tank_ptr_->getWorldPose().x()) < threshold) && // todo same as cubic
	    (abs(posY - tank_ptr_->getWorldPose().y()) < threshold) &&
	    (abs(ghost_util::SmallestAngleDistRad(theta, tank_ptr_->getWorldAngleRad())) < angle_threshold)){
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseTrapezoid: Success");
		return BT::NodeStatus::SUCCESS;
	}

	if(started_){
		auto now = std::chrono::system_clock::now();
		int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
		if(timeout > 0){
			if(time_elapsed > timeout){
				RCLCPP_WARN(node_ptr_->get_logger(), "MoveToPoseTrapezoid Timeout: %i ms elapsed", time_elapsed);
				start_time_ = std::chrono::system_clock::now();
				command_pub_->publish(msg);
				RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseTrapezoid: sent command");
			}
		}
		else{
			if(time_elapsed > abs(timeout)){
				return BT::NodeStatus::SUCCESS;
			}
		}
	}
	else{
		RCLCPP_INFO(node_ptr_->get_logger(), "MoveToPoseTrapezoid: Started");
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

} // namespace ghost_tank