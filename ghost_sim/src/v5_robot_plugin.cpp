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
#include "ghost_util/parsing_util.hpp"
#include <iostream>
#include "eigen3/Eigen/Geometry"

using Eigen::Dynamic;
using Eigen::RowMajor;
using Eigen::Matrix;

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
        std::vector<int> encoder_ports_;
        Eigen::MatrixXd actuator_jacobian_;
        Eigen::MatrixXd sensor_jacobian_;

        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;
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
        impl_->motor_names_ = ghost_util::getVectorFromString<std::string>(sdf->GetElement("motor_names")->Get<std::string>(), ' ');
        impl_->encoder_names_ = ghost_util::getVectorFromString<std::string>(sdf->GetElement("encoder_names")->Get<std::string>(), ' ');
        impl_->joint_names_ = ghost_util::getVectorFromString<std::string>(sdf->GetElement("joint_names")->Get<std::string>(), ' ');
        impl_->encoder_ports_ = ghost_util::getVectorFromString<int>(sdf->GetElement("encoder_ports")->Get<std::string>(), ' ');

        // Load and error check jacobian matrices
        std::vector<double> actuator_jacobian_temp = ghost_util::getVectorFromString<double>(sdf->GetElement("actuator_jacobian")->Get<std::string>(), ' ');
        std::vector<double> sensor_jacobian_temp = ghost_util::getVectorFromString<double>(sdf->GetElement("sensor_jacobian")->Get<std::string>(), ' ');

        if (actuator_jacobian_temp.size() != impl_->motor_names_.size() * impl_->joint_names_.size())
        {
            std::string err_string = "[V5 Robot Plugin], Actuator Jacobian is of incorrect size, cannot proceed!";
            RCLCPP_ERROR(logger, err_string.c_str());
            return;
        }

        if (sensor_jacobian_temp.size() != impl_->encoder_names_.size() * impl_->joint_names_.size())
        {
            std::string err_string = "[V5 Robot Plugin], Sensor Jacobian is of incorrect size, cannot proceed!";
            RCLCPP_ERROR(logger, err_string.c_str());
            return;
        }

        // Populate Eigen Matrices for each jacobian
        impl_->actuator_jacobian_ = Eigen::Map<Matrix<double,Dynamic,Dynamic,RowMajor>>(actuator_jacobian_temp.data(), impl_->joint_names_.size(), impl_->motor_names_.size());
        impl_->sensor_jacobian_ = Eigen::Map<Matrix<double,Dynamic,Dynamic,RowMajor>>(sensor_jacobian_temp.data(), impl_->joint_names_.size(), impl_->encoder_names_.size());

        std::cout << "[V5 Robot Plugin] Actuator Jacobian: \n"<< impl_->actuator_jacobian_ << std::endl;
        std::cout << "[V5 Robot Plugin] Sensor Jacobian: \n"<< impl_->sensor_jacobian_ << std::endl;

        // Create a connection so the OnUpdate function is called at every simulation
        // iteration. Remove this call, the connection and the callback if not needed.
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&V5RobotPlugin::OnUpdate, this));
    }

    void V5RobotPlugin::OnUpdate()
    {
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(V5RobotPlugin)
} // namespace v5_robot_plugin