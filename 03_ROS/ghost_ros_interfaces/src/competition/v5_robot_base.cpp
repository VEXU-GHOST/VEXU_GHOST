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

#include "ghost_ros_interfaces/competition/v5_robot_base.hpp"

#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>
#include "rclcpp/rclcpp.hpp"

using ghost_planners::RobotTrajectory;
using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::RobotHardwareInterface;
using ghost_v5_interfaces::util::loadRobotConfigFromYAMLFile;
using std::placeholders::_1;
namespace ghost_ros_interfaces
{

void V5RobotBase::configure()
{
  std::cout << "Configuring V5 Robot Base!" << std::endl;
  node_ptr_ = std::make_shared<rclcpp::Node>("competition_state_machine_node");

  loadRobotHardwareInterface();

  sensor_update_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::V5SensorUpdate>(
    "/v5/sensor_update",
    10,
    std::bind(&V5RobotBase::sensorUpdateCallback, this, _1)
  );

  actuator_command_pub_ = node_ptr_->create_publisher<ghost_msgs::msg::V5ActuatorCommand>(
    "/v5/actuator_command",
    10);

  trajectory_sub_ = node_ptr_->create_subscription<ghost_msgs::msg::RobotTrajectory>(
    "/motion_planner/trajectory",
    10,
    std::bind(&V5RobotBase::trajectoryCallback, this, _1)
  );

  m_start_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StartRecorder>(
    "bag_recorder/start");

  m_stop_recorder_client = node_ptr_->create_client<ghost_msgs::srv::StopRecorder>(
    "bag_recorder/stop");

  start_time_ = std::chrono::system_clock::now();
  trajectory_start_time_ = 0;

  robot_trajectory_ptr_ = std::make_shared<RobotTrajectory>();

  initialize();
  configured_ = true;
}

void V5RobotBase::loadRobotHardwareInterface()
{

  // ZAARA
  
  // Get YAML path from ROS Param
  node_ptr_->declare_parameter("robot_config_yaml_path", "");
  std::string robot_config_yaml_path =
    node_ptr_->get_parameter("robot_config_yaml_path").as_string();

  // Load RobotHardwareInterface from YAML
  auto device_config_map = loadRobotConfigFromYAMLFile(robot_config_yaml_path);
  rhi_ptr_ = std::make_shared<RobotHardwareInterface>(
    device_config_map,
    hardware_type_e::COPROCESSOR);
}

void V5RobotBase::sensorUpdateCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg)
{
  // Update Competition State Machine
  updateCompetitionState(
    msg->competition_status.is_disabled,
    msg->competition_status.is_autonomous);

  // Update Sensor Data in the Robot Hardware Interface
  fromROSMsg(*rhi_ptr_, *msg);

  // Perform any operations that should happen on every loop
  onNewSensorData();

  try {
    // Execute Competition State Machine
    switch (curr_comp_state_) {
      case robot_state_e::DISABLED:
        disabled();
        break;

      case robot_state_e::AUTONOMOUS:
        autonomous(getTimeFromStart());
        break;

      case robot_state_e::TELEOP:
        teleop(getTimeFromStart());
        break;
    }
  } catch (std::exception & e) {
    RCLCPP_WARN(node_ptr_->get_logger(), e.what());
  } catch (...) {
    std::exception_ptr p = std::current_exception();
    RCLCPP_WARN(node_ptr_->get_logger(), (p ? p.__cxa_exception_type()->name() : "null"));
  }

  // Get Actuator Msg from RobotHardwareInterface and publish
  ghost_msgs::msg::V5ActuatorCommand cmd_msg{};
  cmd_msg.header.stamp = node_ptr_->get_clock()->now();
  toROSMsg(*rhi_ptr_, cmd_msg);
  actuator_command_pub_->publish(cmd_msg);
}

void V5RobotBase::updateCompetitionState(bool is_disabled, bool is_autonomous)
{
  // Update state with new value
  if (is_disabled) {
    curr_comp_state_ = robot_state_e::DISABLED;
  } else if (is_autonomous) {
    curr_comp_state_ = robot_state_e::AUTONOMOUS;
  } else {
    curr_comp_state_ = robot_state_e::TELEOP;
  }

  // Process state transitions
  if ((curr_comp_state_ == robot_state_e::AUTONOMOUS) &&
    (last_comp_state_ != robot_state_e::AUTONOMOUS))
  {
    // DISABLED -> AUTONOMOUS
    start_time_ = std::chrono::system_clock::now();
    // start bag recording
    auto req = std::make_shared<ghost_msgs::srv::StartRecorder::Request>();
    m_start_recorder_client->async_send_request(req);
  }
  if ((curr_comp_state_ == robot_state_e::TELEOP) && (last_comp_state_ != robot_state_e::TELEOP)) {
    // DISABLED/AUTONOMOUS -> TELEOP
    start_time_ = std::chrono::system_clock::now();
  }

  if ((curr_comp_state_ == robot_state_e::DISABLED) &&
    (last_comp_state_ == robot_state_e::TELEOP))
  {
    // TELEOP->DISABLE
    start_time_ = std::chrono::system_clock::now();
    // stop bag recording
    auto req = std::make_shared<ghost_msgs::srv::StopRecorder::Request>();
    m_stop_recorder_client->async_send_request(req);
  }


  last_comp_state_ = curr_comp_state_;
}

double V5RobotBase::getTimeFromStart() const
{
  auto curr_time = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - start_time_).count() /
         1000.0;
}

void V5RobotBase::trajectoryCallback(const ghost_msgs::msg::RobotTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Received Trajectory");
  trajectory_start_time_ = getTimeFromStart();

  if (robot_trajectory_ptr_ == nullptr) {
    robot_trajectory_ptr_ = std::make_shared<RobotTrajectory>();
  }
  fromROSMsg(*robot_trajectory_ptr_, *msg);
}

} // namespace ghost_ros_interfaces
