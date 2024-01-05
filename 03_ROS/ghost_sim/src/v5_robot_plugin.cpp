/*
 * Filename: swerve_odometry_plugin
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Monday October 24th 2022 2:19:23 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include <mutex>
#include <unordered_map>

#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_msgs/msg/v5_encoder_state.hpp"
#include "ghost_msgs/msg/v5_sensor_update.hpp"

#include "ghost_common/util/parsing_util.hpp"
#include "ghost_common/v5_robot_config_defs.hpp"

#include "ghost_control/models/dc_motor_model.hpp"
#include "ghost_control/motor_controller.hpp"

#include "ghost_sim/v5_robot_plugin.hpp"

using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::RowMajor;

using ghost_control::DCMotorModel;
using ghost_control::MotorController;

namespace v5_robot_plugin {

// Class to hold private data members (PIMPL pattern)
class V5RobotPluginPrivate {
public:
	/// Connection to world update event. Callback is called while this is alive.
	gazebo::event::ConnectionPtr update_connection_;

	// Gazebo Ptrs
	gazebo::physics::ModelPtr model_;

	// Parameter Vectors
	std::vector<std::string> motor_names_;
	std::vector<std::string> encoder_names_;
	std::vector<std::string> joint_names_;

	// Incoming joint state data
	std::vector<std::string> joint_msg_list_;
	Eigen::VectorXd joint_positions_;
	Eigen::VectorXd joint_velocities_;
	Eigen::VectorXd joint_efforts_;

	// Outgoing encoder data
	Eigen::VectorXf encoder_vel_data_;
	ghost_msgs::msg::V5EncoderState encoder_msg_;

	std::unordered_map<std::string, std::shared_ptr<DCMotorModel> > motor_model_map_;
	std::unordered_map<std::string, std::shared_ptr<MotorController> > motor_controller_map_;

	Eigen::MatrixXd actuator_jacobian_;
	Eigen::MatrixXd sensor_jacobian_;

	/// Node for ROS communication.
	gazebo_ros::Node::SharedPtr ros_node_;
	rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;
	rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_pub_;

	std::mutex actuator_update_callback_mutex;
};

V5RobotPlugin::V5RobotPlugin() :
	impl_(std::make_unique<V5RobotPluginPrivate>()){
}

V5RobotPlugin::~V5RobotPlugin(){
}

void V5RobotPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf){
	// Get ROS Node and Gazebo Model Ptr
	impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
	auto logger = impl_->ros_node_->get_logger();
	impl_->model_ = model;

	// Initialize ROS Subscriptions
	impl_->actuator_command_sub_ = impl_->ros_node_->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
		"v5/actuator_command",
		10,
		[this](const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg){
			std::unique_lock update_lock(impl_->actuator_update_callback_mutex);
			for(const std::string &name : impl_->motor_names_){
				// Get V5 Motor Command msg, Motor Model, and Motor Controller for each motor
				const auto &motor_cmd_msg = msg->motor_commands[ghost_v5_config::motor_config_map.at(name).port];
				auto motor_model_ptr = impl_->motor_model_map_.at(name);
				auto motor_controller_ptr = impl_->motor_controller_map_.at(name);

				// Update Controller Setpoints
				motor_controller_ptr->setMotorCommand(
					motor_cmd_msg.desired_position,
					motor_cmd_msg.desired_velocity,
					motor_cmd_msg.desired_voltage,
					motor_cmd_msg.desired_torque);

				// Update Control Mode
				motor_controller_ptr->setControlMode(
					motor_cmd_msg.position_control,
					motor_cmd_msg.velocity_control,
					motor_cmd_msg.voltage_control,
					motor_cmd_msg.torque_control);

				// Update Current Limit
				motor_model_ptr->setMaxCurrent(motor_cmd_msg.current_limit);
			}
		});

	// Initialize ROS Publishers
	impl_->sensor_update_pub_ = impl_->ros_node_->create_publisher<ghost_msgs::msg::V5SensorUpdate>(
		"v5/sensor_update",
		10);

	// Check for required parameters
	std::vector<std::string> params{
		"joint_names",
		"motor_names",
		"encoder_names",
		"actuator_jacobian",
		"sensor_jacobian"};

	for(std::string &param : params){
		if(!sdf->HasElement(param)){
			std::string err_string = "[V5 Robot Plugin] Missing <" + param + ">, cannot proceed";
			RCLCPP_ERROR(logger, err_string.c_str());
			return;
		}
	}

	// Parse input plugin parameters
	impl_->joint_names_ = ghost_common::getVectorFromString<std::string>(sdf->GetElement("joint_names")->Get<std::string>(), ' ');
	impl_->motor_names_ = ghost_common::getVectorFromString<std::string>(sdf->GetElement("motor_names")->Get<std::string>(), ' ');
	impl_->encoder_names_ = ghost_common::getVectorFromString<std::string>(sdf->GetElement("encoder_names")->Get<std::string>(), ' ');

	std::vector<double> actuator_jacobian_temp = ghost_common::getVectorFromString<double>(sdf->GetElement("actuator_jacobian")->Get<std::string>(), ' ');
	std::vector<double> sensor_jacobian_temp = ghost_common::getVectorFromString<double>(sdf->GetElement("sensor_jacobian")->Get<std::string>(), ' ');

	// Input Validation
	if(actuator_jacobian_temp.size() != impl_->motor_names_.size() * impl_->joint_names_.size()){
		std::string err_string = "[V5 Robot Plugin], Actuator Jacobian is incorrect size, cannot proceed!";
		RCLCPP_ERROR(logger, err_string.c_str());
		return;
	}

	if(sensor_jacobian_temp.size() != impl_->encoder_names_.size() * impl_->joint_names_.size()){
		std::string err_string = "[V5 Robot Plugin], Sensor Jacobian is incorrect size, cannot proceed!";
		RCLCPP_ERROR(logger, err_string.c_str());
		return;
	}

	// Populate Eigen Matrices for each jacobian
	impl_->actuator_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor> >(actuator_jacobian_temp.data(), impl_->joint_names_.size(), impl_->motor_names_.size());
	impl_->sensor_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor> >(sensor_jacobian_temp.data(), impl_->joint_names_.size(), impl_->encoder_names_.size());

	std::cout << "[V5 Robot Plugin] Actuator Jacobian: \n"
	          << impl_->actuator_jacobian_ << std::endl;
	std::cout << "[V5 Robot Plugin] Sensor Jacobian: \n"
	          << impl_->sensor_jacobian_ << std::endl;

	// // Instantiate Motor Models and Motor Controllers for each motor
	for(const std::string &name : impl_->motor_names_){
		//     // Get config from global definitions
		try{
			ghost_v5_config::MotorConfigStruct config = (ghost_v5_config::motor_config_map.at(name)).config;

			impl_->motor_model_map_[name] = std::make_shared<DCMotorModel>(
				config.motor__nominal_free_speed,
				config.motor__stall_torque,
				config.motor__free_current,
				config.motor__stall_current,
				config.motor__max_voltage,
				config.motor__gear_ratio);

			impl_->motor_controller_map_[name] = std::make_shared<MotorController>(config);
		}
		catch(const std::exception& e){
			throw(std::runtime_error("[V5 Robot Plugin] Error: Motor Name <" + name + ">"));
		}
	}

	// Create a connection so the OnUpdate function is called at every simulation
	// iteration. Remove this call, the connection and the callback if not needed.
	impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&V5RobotPlugin::OnUpdate, this));
}

void V5RobotPlugin::jointToEncoderTransform(){
	int col_index = 0;
	// Lamda for-each expression to get encoder values for every joint
	for_each(begin(impl_->joint_msg_list_), end(impl_->joint_msg_list_), [&](const std::string &joint_data){
			// Get iterator that points to corresponding joint name in joint_names_
			auto joint_name_itr = find(impl_->joint_names_.begin(), impl_->joint_names_.end(), joint_data);

			// Convert iterator to index
			size_t jacobian_row_index = distance(begin(impl_->joint_names_), joint_name_itr);

			// impl_->encoder_vel_data_[col_index] = impl_->model_->GetJoint(joint_data)->Position() * impl_->sensor_jacobian_(jacobian_row_index, col_index);
			col_index++;
			impl_->encoder_msg_ = this->wrapEncoderMsg(col_index);
		});
}

ghost_msgs::msg::V5EncoderState V5RobotPlugin::wrapEncoderMsg(const int col_index){
	auto msg = ghost_msgs::msg::V5EncoderState();
	msg.device_name = impl_->encoder_names_[col_index];
	msg.device_id = 1;
	msg.device_connected = true;
	msg.position_degrees = 30.0;
	// msg.position_degrees = (encoder_vel_data_[col_index] - last_encoder_vel_data_[col_index]) * dt;
	msg.velocity_rpm = impl_->encoder_vel_data_(col_index);
	msg.torque_nm = 0.0;
	msg.voltage_mv = 0.0;
	msg.current_ma = 0.0;
	msg.power_w = 0.0;
	msg.temp_c = 0.0;

	return msg;
}

ghost_msgs::msg::V5SensorUpdate V5RobotPlugin::populateSensorMsg(){
	auto msg = ghost_msgs::msg::V5SensorUpdate();
	// msg.header =;
	// msg.msg_id = 1;
	// msg.encoders =

	return msg;
}

void V5RobotPlugin::getJointStates(){
	int index = 0;
	// Lamda for-each expression to get encoder values for every joint
	for_each(begin(impl_->joint_msg_list_), end(impl_->joint_msg_list_), [&](const std::string &joint_name){
			impl_->joint_positions_(index) = impl_->model_->GetJoint(joint_name)->Position(2);
			impl_->joint_velocities_(index) = impl_->model_->GetJoint(joint_name)->GetVelocity(2);
			impl_->joint_efforts_(index) = impl_->model_->GetJoint(joint_name)->GetForce(2);
			index++;
		});
}

void V5RobotPlugin::OnUpdate(){
	// Acquire msg update lock
	std::unique_lock update_lock(impl_->actuator_update_callback_mutex);

	this->getJointStates();
	std::cout << impl_->joint_positions_ << std::endl;

	// 1) Populate Joint Position and Velocity Vectors
	// 2) Process Sensor Update
	//  2.1) Transform Joint Position/Velocity vectors into actuator/encoder states using Jacobian pseudoinverse
	//  2.3) Populate SensorUpdateMsg
	//  2.4) (Optional) Populate joystick update based on the data from joystick subscriber (That I'm gonna add!)
	//  2.5) Publish
	// 3) Process Actuator Update
	//  3.1) Transform Joint Position/Velocity vectors into actuator space using Jacobian pseudoinverse
	//  3.2) For each motor controller, update position and velocity and async gets setpoints
	//  3.3) For each motor controller, calculate the motor effort (voltage) based on error from 3.2
	//  3.4) For each motor model update velocity from sim
	//  3.5) For each motor model, update the motor effort (voltage)
	//  3.6) Concatenate the simulated torques from each motor model into a vector
	//  3.7) Transform actuator torques to joint torques using  using Jacobian transpose
	//  3.8) Update each joint in gazebo

	// Converts joint data from Gazebo to encoder data using sensor jacobian matrix
	this->jointToEncoderTransform();
	// sensor update pub publishes a sensor msg: wrap encoder_msg again to a sensor msg
	// impl_->sensor_update_pub_->publish(impl_->encoder_msg_);
	this->populateSensorMsg();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(V5RobotPlugin)

}       // namespace v5_robot_plugin