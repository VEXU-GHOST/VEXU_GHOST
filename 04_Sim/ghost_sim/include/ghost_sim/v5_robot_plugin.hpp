/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

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

namespace v5_robot_plugin
{

// Forward declaration of private data class.
class V5RobotPluginPrivate;

class V5RobotPlugin : public gazebo::ModelPlugin
{
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
  std::unique_ptr<V5RobotPluginPrivate> impl_;
};

}       // namespace v5_robot_plugin
