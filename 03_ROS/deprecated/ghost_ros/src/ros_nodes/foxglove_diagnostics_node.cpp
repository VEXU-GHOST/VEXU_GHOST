#include "ghost_ros/ros_nodes/foxglove_diagnostics_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ghost_ros {

FoxgloveDiagnosticsNode::FoxgloveDiagnosticsNode() :
	rclcpp::Node("foxglove_diagnostics_node"){
	declare_parameter<std::vector<std::string> >("motor_names");
	motor_names_ = get_parameter("motor_names").as_string_array();

	// Subscriptions
	v5_sensor_update_sub_ = create_subscription<ghost_msgs::msg::V5SensorUpdate>(
		"v5/sensor_update",
		10,
		std::bind(&FoxgloveDiagnosticsNode::V5SensorUpdateCallback, this, _1));

	v5_actuator_command_sub_ = create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
		"v5/actuator_command",
		10,
		std::bind(&FoxgloveDiagnosticsNode::V5ActuatorCommandCallback, this, _1));

	// Create publisher for each motor
	for(std::string &name : motor_names_){
		motor_state_pubs_[name] = create_publisher<std_msgs::msg::Float32MultiArray>("foxglove/" + name + "/state", 10);
		motor_setpoint_pubs_[name] = create_publisher<std_msgs::msg::Float32MultiArray>("foxglove/" + name + "/setpoint", 10);
	}

	// Publishers
	port_status_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("foxglove/v5_ports", 10);
	is_connected_pub_ = create_publisher<std_msgs::msg::Bool>("foxglove/is_connected", 10);
	is_autonomous_pub_ = create_publisher<std_msgs::msg::Bool>("foxglove/is_autonomous", 10);
	is_teleop_pub_ = create_publisher<std_msgs::msg::Bool>("foxglove/is_teleop", 10);

	// Timer for Connection timeout
	port_timer_ = this->create_wall_timer(
		250ms,
		[this](){
			if(std::chrono::system_clock::now() - last_port_update_time_ > 250ms){
				auto port_status_msg = std_msgs::msg::Float32MultiArray{};
				port_status_msg.data.resize(21, 0);
				port_status_pub_->publish(port_status_msg);
			}
		});

	v5_hearbeat_toggle_ = false;
}

void FoxgloveDiagnosticsNode::V5ActuatorCommandCallback(const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg){
	// Publish motor state
	for(auto &cmd : msg->motor_commands){
		if(motor_setpoint_pubs_.count(cmd.motor_name)){
			// Update motor msg
			auto motor_setpoint_msg = std_msgs::msg::Float32MultiArray{};
			motor_setpoint_msg.data.push_back(cmd.desired_position);
			motor_setpoint_msg.data.push_back(cmd.desired_velocity);
			motor_setpoint_msg.data.push_back(cmd.desired_voltage);
			motor_setpoint_pubs_[cmd.motor_name]->publish(motor_setpoint_msg);
		}
	}
}

void FoxgloveDiagnosticsNode::V5SensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
	// Toggle Hearbeat display at reasonable frequency
	if(std::chrono::system_clock::now() - last_heartbeat_update_time_ > 250ms){
		v5_hearbeat_toggle_ = !v5_hearbeat_toggle_;
		last_heartbeat_update_time_ = std::chrono::system_clock::now();
	}

	// Handle Competition State
	auto is_teleop_msg = std_msgs::msg::Bool{};
	is_teleop_msg.data = !msg->is_disabled && !msg->is_autonomous;
	is_teleop_pub_->publish(is_teleop_msg);

	auto is_auto_msg = std_msgs::msg::Bool{};
	is_auto_msg.data = !msg->is_disabled && msg->is_autonomous;
	is_autonomous_pub_->publish(is_auto_msg);

	auto is_connected_msg = std_msgs::msg::Bool{};
	is_connected_msg.data = msg->is_connected;
	is_connected_pub_->publish(is_connected_msg);

	auto port_status_msg = std_msgs::msg::Float32MultiArray{};
	port_status_msg.data.push_back(v5_hearbeat_toggle_);

	// Publish motor state
	for(int i = 1; i < 21; i++){
		auto encoder = msg->encoders[i];
		port_status_msg.data.push_back(encoder.device_connected);

		if(motor_state_pubs_.count(encoder.device_name)){
			// Update motor msg
			auto motor_state_msg = std_msgs::msg::Float32MultiArray{};
			motor_state_msg.data.push_back(encoder.position_degrees);
			motor_state_msg.data.push_back(encoder.velocity_rpm);
			motor_state_msg.data.push_back(encoder.current_ma);
			motor_state_msg.data.push_back(encoder.voltage_mv);
			motor_state_msg.data.push_back(encoder.temp_c);
			motor_state_msg.data.push_back(encoder.power_w);
			motor_state_pubs_[encoder.device_name]->publish(motor_state_msg);
		}
	}

	port_status_pub_->publish(port_status_msg);
	last_port_update_time_ = std::chrono::system_clock::now();
}

}

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ghost_ros::FoxgloveDiagnosticsNode>());
	rclcpp::shutdown();
	return 0;
}