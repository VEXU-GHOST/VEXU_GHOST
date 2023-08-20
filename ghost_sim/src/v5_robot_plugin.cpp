/*
 * Filename: swerve_odometry_plugin
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Monday October 24th 2022 2:19:23 pm
 * Modified By: Maxx Wilson
 */

#include "v5_robot_plugin.hpp"

#include <iostream>
#include <unordered_map>
#include "eigen3/Eigen/Geometry"

#include "ghost_msgs/msg/v5_sensor_update.hpp"
#include "ghost_msgs/msg/v5_actuator_command.hpp"
#include "ghost_common/util/parsing_util.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::RowMajor;

namespace v5_robot_plugin
{

    // Class to hold private data members (PIMPL pattern)
    class V5RobotPluginPrivate
    {
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
        std::vector<double> joint_positions_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_efforts_;

        std::unordered_map<std::string, int> motor_port_map_;
        std::unordered_map<std::string, int> encoder_port_map_;

        Eigen::MatrixXd actuator_jacobian_;
        Eigen::MatrixXd sensor_jacobian_;

        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;
        rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Publisher<ghost_msgs::msg::V5SensorUpdate>::SharedPtr sensor_update_pub_;
    };

    V5RobotPlugin::V5RobotPlugin()
        : impl_(std::make_unique<V5RobotPluginPrivate>())
    {
    }

    V5RobotPlugin::~V5RobotPlugin()
    {
    }

    void V5RobotPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        // Get ROS Node and Gazebo Model Ptr
        impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
        auto logger = impl_->ros_node_->get_logger();
        impl_->model_ = model;

        // Initialize ROS Subscriptions
        impl_->actuator_command_sub_ = impl_->ros_node_->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
            "v5/actuator_commands",
            10,
            [this](const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
            {
                // 1) Iterate through motor_names and use motor_port_map to get motor command
            });
        
        impl_->joint_state_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                impl_->joint_msg_list_ = msg->name;
                impl_->joint_positions_ = msg->position;
                impl_->joint_velocities_ = msg->velocity;
                impl_->joint_efforts_ = msg->effort;
            });

        // Initialize ROS Publishers
        impl_->sensor_update_pub_ = impl_->ros_node_->create_publisher<ghost_msgs::msg::V5SensorUpdate>(
            "v5/sensor_update",
            10);

        // Check for required parameters
        std::vector<std::string> params{
            "motor_names",
            "encoder_names",
            "encoder_ports",
            "joint_names",
            "actuator_jacobian",
            "sensor_jacobian"};

        for (std::string &param : params)
        {
            if (!sdf->HasElement(param))
            {
                std::string err_string = "[V5 Robot Plugin] Missing <" + param + ">, cannot proceed";
                RCLCPP_ERROR(logger, err_string.c_str());
                return;
            }
        }

        // Parse input plugin parameters
        impl_->joint_names_ = ghost_common::getVectorFromString<std::string>(sdf->GetElement("joint_names")->Get<std::string>(), ' ');

        impl_->motor_names_ = ghost_common::getVectorFromString<std::string>(sdf->GetElement("motor_names")->Get<std::string>(), ' ');
        auto motor_port_vector = ghost_common::getVectorFromString<int>(sdf->GetElement("motor_ports")->Get<std::string>(), ' ');

        impl_->encoder_names_ = ghost_common::getVectorFromString<std::string>(sdf->GetElement("encoder_names")->Get<std::string>(), ' ');
        auto encoder_port_vector = ghost_common::getVectorFromString<int>(sdf->GetElement("encoder_ports")->Get<std::string>(), ' ');

        std::vector<double> actuator_jacobian_temp = ghost_common::getVectorFromString<double>(sdf->GetElement("actuator_jacobian")->Get<std::string>(), ' ');
        std::vector<double> sensor_jacobian_temp = ghost_common::getVectorFromString<double>(sdf->GetElement("sensor_jacobian")->Get<std::string>(), ' ');

        // Input Validation
        if (motor_port_vector.size() != impl_->motor_names_.size())
        {
            std::string err_string = "[V5 Robot Plugin], motor_names and motor_ports are different sizes!";
            RCLCPP_ERROR(logger, err_string.c_str());
            return;
        }

        if (encoder_port_vector.size() != impl_->encoder_names_.size())
        {
            std::string err_string = "[V5 Robot Plugin], encoder_names and encoder_ports are different sizes!";
            RCLCPP_ERROR(logger, err_string.c_str());
            return;
        }

        if (actuator_jacobian_temp.size() != impl_->motor_names_.size() * impl_->joint_names_.size())
        {
            std::string err_string = "[V5 Robot Plugin], Actuator Jacobian is incorrect size, cannot proceed!";
            RCLCPP_ERROR(logger, err_string.c_str());
            return;
        }

        if (sensor_jacobian_temp.size() != impl_->encoder_names_.size() * impl_->joint_names_.size())
        {
            std::string err_string = "[V5 Robot Plugin], Sensor Jacobian is incorrect size, cannot proceed!";
            RCLCPP_ERROR(logger, err_string.c_str());
            return;
        }

        // Populate port maps for motors and encoders
        for (int i = 0; i < impl_->motor_names_.size(); i++)
        {
            auto name = impl_->motor_names_[i];
            auto port = motor_port_vector[i];
            impl_->motor_port_map_[name] = port;
        }

        for (int i = 0; i < impl_->encoder_names_.size(); i++)
        {
            auto name = impl_->encoder_names_[i];
            auto port = encoder_port_vector[i];
            impl_->encoder_port_map_[name] = port;
        }

        // Populate Eigen Matrices for each jacobian
        impl_->actuator_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(actuator_jacobian_temp.data(), impl_->joint_names_.size(), impl_->motor_names_.size());
        impl_->sensor_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(sensor_jacobian_temp.data(), impl_->joint_names_.size(), impl_->encoder_names_.size());

        std::cout << "[V5 Robot Plugin] Actuator Jacobian: \n"
                  << impl_->actuator_jacobian_ << std::endl;
        std::cout << "[V5 Robot Plugin] Sensor Jacobian: \n"
                  << impl_->sensor_jacobian_ << std::endl;

        // Create a connection so the OnUpdate function is called at every simulation
        // iteration. Remove this call, the connection and the callback if not needed.
        // impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        //     std::bind(&V5RobotPlugin::OnUpdate, this));

    }

    void V5RobotPlugin::jointToEncoderTransform(){
        // Lamda for-each expression to get encoder values for every joint
        // for_each(begin(impl_->joint_msg_list_), end(impl_->joint_msg_list_), [&](sensor_msgs::msg::JointState& joint_data){
        //     // get encoder Jacobian row index in encoder_names given joint_data name
        //     std::string jacobian_row_index = impl_->joint_names_.indexOf(joint_data.name);
            
        //     // 
        //     encoder_data = this->populate_encoder_data(loop_index);
        // });
    }

    void V5RobotPlugin::OnUpdate()
    {
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(V5RobotPlugin)
} // namespace v5_robot_plugin