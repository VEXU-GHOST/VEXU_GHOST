#ifndef GHOST_SIM__EXAMPLES_GAZEBO_PLUGIN_HPP_
#define GHOST_SIM__EXAMPLES_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include "std_msgs/msg/float32.hpp"

// For std::unique_ptr, could be removed
#include <memory>

// namespace gazebo
// {

// // Forward declaration of private data class.
// class TankMovePrivate;

// class TankMove : public gazebo::ModelPlugin
// {
// public:
//   /// Constructor
//   TankMove();

//   /// Destructor
//   ~TankMove();

//   void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

// protected:
//   /// Optional callback to be called at every simulation iteration.
//   void OnUpdate();

// private:
//   /// Recommended PIMPL pattern. This variable should hold all private
//   /// data members.
//   std::unique_ptr<TankMovePrivate> impl_;
// };

// }  

// #endif  



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

namespace gazebo
{

// Forward declaration of private data class.
class TankMovePrivate;

class TankMove : public gazebo::ModelPlugin
{
public:
  /// Constructor
  TankMove();

  /// Destructor
  ~TankMove();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  std::shared_ptr<ghost_v5_interfaces::RobotHardwareInterface> rhi_ptr_;
  /**
   *    Converts joint data from Gazebo to encoder data using sensor jacobian matrix
   */
  void jointToEncoderTransform();

  /**
   * Transforms motor data (motor pos, or vel, or accel), to the location of the corresponding joint in swerve model
   */
  Eigen::VectorXd motorToJointTransform(const Eigen::VectorXd & motor_data);

  /**
   * Transforms joint data (joint pos, or vel, or accel), to the location of the corresponding motor in swerve model
   */
  Eigen::VectorXd jointToMotorTransform(const Eigen::VectorXd & joint_data);

  /**
   * Updates joint commands given motor actuator input command
   */
  void updateMotorController();

  /**
   * Wraps encoder matrix into V5Encoder state msg[]
   */
  void wrapEncoderMsg();

  /**
   * Populates sensor msg using current encoder msg
   */
  void populateSensorMsg();

  /**
   * Updates joint angles and velocities from gazebo
   */
  void updateJointStates();

  /**
   * Applies command joint torques to gazebo links
   */
  void applySimJointTorques();

  /**
   * Optional callback to be called by Gazebo at every simulation iteration.
   */
  void OnUpdate();

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<TankMovePrivate> impl_;
};

}       // namespace v5_robot_plugin

