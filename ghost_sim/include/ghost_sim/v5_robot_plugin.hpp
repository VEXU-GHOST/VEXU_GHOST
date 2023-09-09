#pragma once

#include <algorithm>
#include <Eigen/QR>
#include <memory>
#include <random>
#include <vector>
#include <math.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace v5_robot_plugin {

// Forward declaration of private data class.
class V5RobotPluginPrivate;

class V5RobotPlugin : public gazebo::ModelPlugin {
public:
	/// Constructor
	V5RobotPlugin();

	/// Destructor
	~V5RobotPlugin();

	/// Gazebo calls this when the plugin is loaded.
	/// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
	/// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
	/// `gazebo::rendering::VisualPtr`, etc.
	/// \param[in] sdf SDF element containing user-defined parameters.
	void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
	void jointToEncoderTransform();
	Eigen::VectorXd motorToJointTransform(Eigen::VectorXd motor_data);
	void updateMotorController();
	void wrapEncoderMsg();
	void populateSensorMsg();
	void getJointStates();
	void applySimJointTorques();

	/// Optional callback to be called at every simulation iteration.
	void OnUpdate();

private:
	/// Recommended PIMPL pattern. This variable should hold all private
	/// data members.
	std::unique_ptr<V5RobotPluginPrivate> impl_;
};

}       // namespace v5_robot_plugin