#include "simulator_plugins/odometry_plugin.hpp"

namespace odometry_plugin
{

    class OdometryPluginPrivate : public SensorPlugin
    {
        public:
        //TODO: members go here

    };

    OdometryPlugin::OdometryPlugin()
    : impl_(std::make_unique<OdometryPluginPrivate>())
    {
    }

    OdometryPlugin::~OdometryPlugin()
    {
    }

    void OdometryPlugin::Load(gazebo::physics::SensorPtr sensor, sdf::ElementPtr sdf)
    {
    }

    void OdometryPlugin::OnUpdate()
    {
        // 
        auto torque_msg = std_msgs::msg::Float32{};
        torque_msg.data = impl_->motor_model_.getTorqueOutput();
        impl_->output_torque_pub_->publish(torque_msg);

    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin)
}  // namespace gazebo_swerve_plugin