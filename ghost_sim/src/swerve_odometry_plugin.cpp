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
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr odom_pub_;

    double x_stddev;
    double y_stddev;
    double r_stddev;

    std::normal_distribution<double> x_noise_dist;
    std::normal_distribution<double> y_noise_dist;
    std::normal_distribution<double> r_noise_dist;

    // random device class instance, source of 'true' randomness for initializing random seed
    std::random_device rd; 

    // Mersenne twister PRNG, initialized with seed from previous random device instance
    std::mt19937 gen; 
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

    // Check for valid plugin config
    std::vector<std::string> params{
        "x_stddev",
        "y_stddev",
        "r_stddev",
    };

    for(std::string& param: params){
    if(!sdf->HasElement(param)){
        RCLCPP_ERROR(logger, "Odometry plugin missing <" + param + ">, cannot proceed");
        return;
    }
    }

    // Get plugin config
    impl_->x_stddev = sdf->GetElement("x_stddev")->Get<double>();
    impl_->y_stddev = sdf->GetElement("y_stddev")->Get<double>();
    impl_->r_stddev = sdf->GetElement("r_stddev")->Get<double>();

    impl_->x_noise_dist = std::normal_distribution<double>(0, impl_->x_stddev);
    impl_->y_noise_dist = std::normal_distribution<double>(0, impl_->y_stddev);
    impl_->r_noise_dist = std::normal_distribution<double>(0, impl_->r_stddev);

    // random device class instance, source of 'true' randomness for initializing random seed
    impl_->gen = std::mt19937(impl_->rd()); 

    // Initialize Publisher
    impl_->odom_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/odom", 10);

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&SwerveOdometryPlugin::OnUpdate, this));
}

void SwerveOdometryPlugin::OnUpdate()
{
    ignition::math::Pose3d world_pose = impl_->model_->WorldPose();

    

    // double angle_error = fmod(impl_->angle_setpoint_, 360) - joint_angle_deg;
    // double angle_sign = std::abs(angle_error)/angle_error;
    // angle_error = std::abs(angle_error) > 180 ? angle_sign*360 - angle_error : angle_error; 

    // Publish current joint torque
    auto odom_msg = geometry_msgs::msg::Vector3Stamped{};
    odom_msg.header.stamp = impl_->ros_node_->get_clock()->now();
    odom_msg.vector.x = world_pose.Pos().X() + impl_->x_noise_dist(impl_->gen);
    odom_msg.vector.y = world_pose.Pos().Y() + impl_->y_noise_dist(impl_->gen);
    odom_msg.vector.z = 2.0 * atan2(world_pose.Rot().Z(), world_pose.Rot().W()) + impl_->r_noise_dist(impl_->gen);

    impl_->odom_pub_->publish(odom_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SwerveOdometryPlugin)
}  // namespace gazebo_swerve_plugin