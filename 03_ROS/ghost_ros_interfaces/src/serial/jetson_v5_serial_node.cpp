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

#include <ghost_ros_interfaces/msg_helpers/msg_helpers.hpp>
#include <ghost_ros_interfaces/serial/jetson_v5_serial_node.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>

using ghost_ros_interfaces::msg_helpers::fromROSMsg;
using ghost_ros_interfaces::msg_helpers::toROSMsg;
using ghost_v5_interfaces::devices::hardware_type_e;
using ghost_v5_interfaces::RobotHardwareInterface;
using ghost_v5_interfaces::util::loadRobotConfigFromYAMLFile;
using std::placeholders::_1;

using namespace std::literals::chrono_literals;
namespace ghost_ros_interfaces
{

JetsonV5SerialNode::JetsonV5SerialNode()
: Node("ghost_serial_node"),
  serial_open_(false),
  using_backup_port_(false)
{
  // Load ROS Params
  declare_parameter("use_checksum", true);
  use_checksum_ = get_parameter("use_checksum").as_bool();

  declare_parameter("verbose", false);
  verbose_ = get_parameter("verbose").as_bool();

  declare_parameter("read_msg_start_seq", "sout");
  read_msg_start_seq_ = get_parameter("read_msg_start_seq").as_string();

  declare_parameter("write_msg_start_seq", "msg");
  write_msg_start_seq_ = get_parameter("write_msg_start_seq").as_string();

  declare_parameter("port_name", "/dev/ttyACM1");
  port_name_ = get_parameter("port_name").as_string();

  declare_parameter("backup_port_name", "/dev/ttyACM2");
  backup_port_name_ = get_parameter("backup_port_name").as_string();

  declare_parameter("robot_config_yaml_path", "");
  std::string robot_config_yaml_path = get_parameter("robot_config_yaml_path").as_string();

  // Load Robot Configuration
  auto device_config_map = loadRobotConfigFromYAMLFile(robot_config_yaml_path);
  rhi_ptr_ = std::make_shared<RobotHardwareInterface>(
    device_config_map,
    hardware_type_e::COPROCESSOR);
  actuator_command_msg_len_ = rhi_ptr_->getActuatorCommandMsgLength();
  sensor_update_msg_len_ = rhi_ptr_->getSensorUpdateMsgLength();
  sensor_update_msg_ = std::vector<unsigned char>(sensor_update_msg_len_, 0);

  // Debug Info
  RCLCPP_INFO(get_logger(), "Port Name: %s", port_name_.c_str());
  RCLCPP_INFO(get_logger(), "Backup Port Name: %s", backup_port_name_.c_str());

  int incoming_packet_len = sensor_update_msg_len_ +
    use_checksum_ +
    read_msg_start_seq_.size() +
    2;                               // Cobs Encoding adds two bytes

  RCLCPP_INFO(get_logger(), "Actuator Command Msg Length: %d", actuator_command_msg_len_);
  RCLCPP_INFO(get_logger(), "State Update Msg Length: %d", sensor_update_msg_len_);
  RCLCPP_INFO(get_logger(), "Incoming Packet Length: %d", incoming_packet_len);

  // Serial Interface
  serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
    write_msg_start_seq_,
    read_msg_start_seq_,
    sensor_update_msg_len_,
    use_checksum_,
    verbose_);

  // Sensor Update Msg Publisher
  sensor_update_pub_ = create_publisher<ghost_msgs::msg::V5SensorUpdate>("v5/sensor_update", 10);

  // Actuator Command Msg Subscriber
  actuator_command_sub_ = create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "v5/actuator_command",
    10,
    std::bind(&JetsonV5SerialNode::actuatorCommandCallback, this, _1));

  // Start Serial Thread
  serial_thread_ = std::thread(&JetsonV5SerialNode::serialLoop, this);
  serial_timeout_thread_ = std::thread(&JetsonV5SerialNode::serialTimeoutLoop, this);
}

JetsonV5SerialNode::~JetsonV5SerialNode()
{
  serial_thread_.join();
  serial_timeout_thread_.join();
}

bool JetsonV5SerialNode::initSerial()
{
  // Wait for serial to become available
  static int err_count = 0;
  try {
    if (!using_backup_port_) {
      RCLCPP_DEBUG(get_logger(), "Attempting to open %s", port_name_.c_str());
      serial_open_ = serial_base_interface_->trySerialInit(port_name_);
    } else {
      RCLCPP_DEBUG(get_logger(), "Attempting to open %s", backup_port_name_.c_str());
      serial_open_ = serial_base_interface_->trySerialInit(backup_port_name_);
    }
  } catch (const std::exception & e) {
    // Throttle error output
    if (((err_count % 50) == 0) || (err_count == 0)) {
      RCLCPP_ERROR(get_logger(), e.what());
    }
    err_count++;
  }

  if (serial_open_) {
    err_count = 0;             // Reset error output if we succeed
    auto opened_port_name = (using_backup_port_) ? backup_port_name_.c_str() : port_name_.c_str();
    RCLCPP_DEBUG(get_logger(), "Succesfully opened serial on port: %s", opened_port_name);
  } else {
    using_backup_port_ = !using_backup_port_;
  }
  return serial_open_;
}

void JetsonV5SerialNode::serialTimeoutLoop()
{
  while (rclcpp::ok()) {
    if ((std::chrono::system_clock::now() - last_msg_time_ > 100ms) && serial_open_) {
      // Acquire exclusive access to serial port, and then reset
      std::unique_lock<std::mutex> serial_lock(serial_reset_mutex_);
      serial_open_ = false;

      // Serial Interface
      serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
        write_msg_start_seq_,
        read_msg_start_seq_,
        sensor_update_msg_len_,
        use_checksum_,
        verbose_);

      serial_lock.unlock();
    }
  }
}

void JetsonV5SerialNode::serialLoop()
{
  while (rclcpp::ok()) {
    // Ensure serial timeout does not interrupt a msg read operation
    std::unique_lock<std::mutex> serial_lock(serial_reset_mutex_);
    if (serial_open_) {
      RCLCPP_DEBUG(get_logger(), "Serial Loop is Running");
      try {
        int msg_len;
        bool msg_found = serial_base_interface_->readMsgFromSerial(sensor_update_msg_, msg_len);

        if (msg_found) {
          RCLCPP_DEBUG(get_logger(), "Received new message over serial");
          last_msg_time_ = std::chrono::system_clock::now();
          publishV5SensorUpdate(sensor_update_msg_);
        }
      } catch (std::exception & e) {
        RCLCPP_ERROR(get_logger(), e.what());
      }
    } else {
      RCLCPP_DEBUG(get_logger(), "Initializing Serial");
      initSerial();
      std::this_thread::sleep_for(10ms);
    }
    serial_lock.unlock();
  }
}

void JetsonV5SerialNode::actuatorCommandCallback(
  const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Received Actuator Command");

  if (!serial_open_) {
    RCLCPP_ERROR(get_logger(), "Cannot write to serial, port is not open");
    return;
  }

  fromROSMsg(*rhi_ptr_, *msg);

  auto msg_buffer = rhi_ptr_->serialize();

  serial_base_interface_->writeMsgToSerial(msg_buffer.data(), actuator_command_msg_len_);
}

void JetsonV5SerialNode::publishV5SensorUpdate(const std::vector<unsigned char> & buffer)
{
  RCLCPP_DEBUG(get_logger(), "Publishing Sensor Update");

  // Update hardware interface
  rhi_ptr_->deserialize(buffer);

  // Initialize msg and set time
  ghost_msgs::msg::V5SensorUpdate sensor_update_msg{};
  auto curr_ros_time = get_clock()->now();
  sensor_update_msg.header.stamp = curr_ros_time;

  // Convert updated RHI to msg
  toROSMsg(*rhi_ptr_, sensor_update_msg);

  // Publish update
  sensor_update_pub_->publish(sensor_update_msg);
}

} // namespace ghost_ros_interfaces

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto serial_node = std::make_shared<ghost_ros_interfaces::JetsonV5SerialNode>();
  rclcpp::spin(serial_node);
  rclcpp::shutdown();
  return 0;
}
