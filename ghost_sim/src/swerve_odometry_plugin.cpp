/*
 * Filename: swerve_odometry_plugin
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday October 24th 2022 2:19:23 pm
 * Modified By: Maxx Wilson
 */

#include "swerve_odometry_plugin.hpp"

namespace swerve_odometry_plugin
{

// Class to hold private data members (PIMPL pattern)
class SwerveOdometryPluginPrivate
{
public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    // Gazebo Ptrs
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr joint_;
    gazebo::physics::LinkPtr link_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr odom_pub_;
};

SwerveOdometryPlugin::SwerveOdometryPlugin()
: impl_(std::make_unique<SwerveOdometryPluginPrivate>())
{
}

SwerveOdometryPlugin::~SwerveOdometryPlugin()
{
}

void SwerveOdometryPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Get ROS Node and Gazebo Model Ptr
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    auto logger = impl_->ros_node_->get_logger();
    impl_->model_= model;

    // // Check for valid plugin config
    // std::vector<std::string> params{
    // };

    // for(std::string& param: params){
    // if(!sdf->HasElement(param)){
    //     RCLCPP_ERROR(logger, "Odometry plugin missing <" + param + ">, cannot proceed");
    //     return;
    // }
    // }

    // Initialize Publisher
    impl_->odom_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3>("/odom", 10);

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&SwerveOdometryPlugin::OnUpdate, this));
}

void SwerveOdometryPlugin::OnUpdate()
{
    // auto world_pose = impl_->model_->WorldPose();
    // RCLCPP_INFO(impl_->ros_node_->get_logger(), "Running");

    // double angle_error = fmod(impl_->angle_setpoint_, 360) - joint_angle_deg;
    // double angle_sign = std::abs(angle_error)/angle_error;
    // angle_error = std::abs(angle_error) > 180 ? angle_sign*360 - angle_error : angle_error; 

    // Publish current joint torque
    auto odom_msg = geometry_msgs::msg::Vector3{};
    odom_msg.x = 0.0;
    odom_msg.y = 0.0;
    odom_msg.z = 0.0;
    impl_->odom_pub_->publish(odom_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SwerveOdometryPlugin)
}  // namespace gazebo_swerve_plugin