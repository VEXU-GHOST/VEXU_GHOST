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

#include <cstring>

#include <ghost_ros_interfaces/serial/jetson_sensor_host_serial_node.hpp>

using namespace std::literals::chrono_literals;

namespace ghost_ros_interfaces
{

namespace
{

// Wire protocol constants — mirror of ghost_sensor_host/.../inc/comms.h.
// The Pico writes one COBS-framed packet per poll batch:
//   COBS( [magic "orinin"][cmd:1][len_lo:1][len_hi:1][payload:N][checksum:1] )
// ghost_serial strips the magic start sequence and COBS, leaving the decoded
// body [cmd][len_lo][len_hi][payload][checksum] in the read buffer.
constexpr uint8_t CMD_DATA = 0x04u;

// Generous upper bound on a decoded packet body (matches COMMS_MAX_OUT_MSG_LEN).
constexpr int READ_BUFFER_LEN = 28210;

// Packed frames as laid out in the CMD_DATA payload (see comms.h).
#pragma pack(push, 1)
struct DataHeader
{
  uint32_t poll_count;
  uint8_t color_cnt;
  uint8_t imu_cnt;
  uint8_t distance_cnt;
  uint8_t gpio_cnt;
};

struct ColorFrame
{
  uint16_t r, g, b;
  uint32_t lux, cct;
};
#pragma pack(pop)

}  // namespace

JetsonSensorHostSerialNode::JetsonSensorHostSerialNode()
: Node("ghost_sensor_host_serial_node"),
  serial_open_(false),
  using_backup_port_(false)
{
  // Load ROS Params
  declare_parameter("use_checksum", false);
  use_checksum_ = get_parameter("use_checksum").as_bool();

  declare_parameter("verbose", false);
  verbose_ = get_parameter("verbose").as_bool();

  // Pico write magic ("orinin") / Jetson write magic ("hostin") from comms.h.
  declare_parameter("read_msg_start_seq", "orinin");
  read_msg_start_seq_ = get_parameter("read_msg_start_seq").as_string();

  declare_parameter("write_msg_start_seq", "hostin");
  write_msg_start_seq_ = get_parameter("write_msg_start_seq").as_string();

  declare_parameter("port_name", "/dev/ttyACM1");
  port_name_ = get_parameter("port_name").as_string();

  declare_parameter("backup_port_name", "/dev/ttyACM2");
  backup_port_name_ = get_parameter("backup_port_name").as_string();

  read_buffer_len_ = READ_BUFFER_LEN;
  read_buffer_ = std::vector<unsigned char>(read_buffer_len_, 0);

  // Debug Info
  RCLCPP_INFO(get_logger(), "Port Name: %s", port_name_.c_str());
  RCLCPP_INFO(get_logger(), "Backup Port Name: %s", backup_port_name_.c_str());

  // Serial Interface
  serial_base_interface_ = std::make_shared<ghost_serial::JetsonSerialBase>(
    write_msg_start_seq_,
    read_msg_start_seq_,
    read_buffer_len_,
    use_checksum_,
    verbose_);

  // Sensor state publishers
  color_sensor_update_pub_ = create_publisher<ghost_msgs::msg::ColorSensorState>(
    "sensor_host/color_sensor_update", rclcpp::SensorDataQoS());
  distance_sensor_update_pub_ = create_publisher<ghost_msgs::msg::DistanceSensorState>(
    "sensor_host/distance_sensor_update", rclcpp::SensorDataQoS());
  imu_update_pub_ = create_publisher<ghost_msgs::msg::ImuState>(
    "sensor_host/imu_update", rclcpp::SensorDataQoS());
  io_expander_update_pub_ = create_publisher<ghost_msgs::msg::IOExpanderState>(
    "sensor_host/io_expander_update", rclcpp::SensorDataQoS());

  // Start Serial Threads
  serial_thread_ = std::thread(&JetsonSensorHostSerialNode::serialLoop, this);
  serial_timeout_thread_ = std::thread(&JetsonSensorHostSerialNode::serialTimeoutLoop, this);
}

JetsonSensorHostSerialNode::~JetsonSensorHostSerialNode()
{
  serial_thread_.join();
  serial_timeout_thread_.join();
}

bool JetsonSensorHostSerialNode::initSerial()
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

void JetsonSensorHostSerialNode::serialTimeoutLoop()
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
        read_buffer_len_,
        use_checksum_,
        verbose_);

      serial_lock.unlock();
    }
  }
}

void JetsonSensorHostSerialNode::serialLoop()
{
  while (rclcpp::ok()) {
    // Ensure serial timeout does not interrupt a msg read operation
    std::unique_lock<std::mutex> serial_lock(serial_reset_mutex_);
    if (serial_open_) {
      RCLCPP_DEBUG(get_logger(), "Serial Loop is Running");
      try {
        int msg_len;
        bool msg_found = serial_base_interface_->readMsgFromSerial(read_buffer_, msg_len);

        if (msg_found) {
          last_msg_time_ = std::chrono::system_clock::now();

          // Decoded body: [cmd][len_lo][len_hi][payload...][checksum]
          if (msg_len < 3) {
            RCLCPP_WARN(get_logger(), "Serial msg too short (%d bytes)", msg_len);
          } else {
            uint8_t cmd = read_buffer_[0];
            uint16_t payload_len =
              static_cast<uint16_t>(read_buffer_[1]) |
              (static_cast<uint16_t>(read_buffer_[2]) << 8);

            if (cmd != CMD_DATA) {
              RCLCPP_DEBUG(get_logger(), "Ignoring non-DATA cmd 0x%02x", cmd);
            } else if (3 + static_cast<int>(payload_len) > msg_len) {
              RCLCPP_WARN(
                get_logger(), "Payload len %u exceeds msg len %d", payload_len, msg_len);
            } else {
              std::vector<unsigned char> payload(
                read_buffer_.begin() + 3, read_buffer_.begin() + 3 + payload_len);
              publishColorSensorUpdate(payload);
              // TODO: dispatch distance/imu/io_expander once implemented.
            }
          }
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

void JetsonSensorHostSerialNode::publishColorSensorUpdate(const std::vector<unsigned char> & payload)
{
  if (payload.size() < sizeof(DataHeader)) {
    RCLCPP_WARN(get_logger(), "DATA payload smaller than header");
    return;
  }

  DataHeader header;
  std::memcpy(&header, payload.data(), sizeof(DataHeader));

  if (header.color_cnt == 0 || header.poll_count == 0) {
    return;
  }

  // Publish the most recent poll round for each color sensor. Frames are laid
  // out as ColorFrame[poll_count][color_cnt] immediately after the header.
  const uint32_t latest_round = header.poll_count - 1;
  for (uint8_t sensor = 0; sensor < header.color_cnt; sensor++) {
    const size_t offset = sizeof(DataHeader) +
      (static_cast<size_t>(latest_round) * header.color_cnt + sensor) * sizeof(ColorFrame);
    if (offset + sizeof(ColorFrame) > payload.size()) {
      RCLCPP_WARN(get_logger(), "Color frame %u out of bounds", sensor);
      break;
    }

    ColorFrame frame;
    std::memcpy(&frame, payload.data() + offset, sizeof(ColorFrame));

    ghost_msgs::msg::ColorSensorState msg;
    msg.name = "color_sensor_" + std::to_string(sensor + 1);
    msg.r = frame.r;
    msg.g = frame.g;
    msg.b = frame.b;
    msg.lux = frame.lux;
    msg.cct = frame.cct;
    color_sensor_update_pub_->publish(msg);
  }
}

void JetsonSensorHostSerialNode::publishDistanceSensorUpdate(const std::vector<unsigned char> & payload)
{
  // TODO: parse DistanceFrame[poll_count][distance_cnt] and publish.
  (void) payload;
}

void JetsonSensorHostSerialNode::publishImuUpdate(const std::vector<unsigned char> & payload)
{
  // TODO: parse ImuFrame[poll_count][imu_cnt] and publish.
  (void) payload;
}

void JetsonSensorHostSerialNode::publishIOExpanderUpdate(const std::vector<unsigned char> & payload)
{
  // TODO: parse GpioFrame[poll_count][gpio_cnt] and publish.
  (void) payload;
}

}  // namespace ghost_ros_interfaces

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto serial_node = std::make_shared<ghost_ros_interfaces::JetsonSensorHostSerialNode>();
  rclcpp::spin(serial_node);
  rclcpp::shutdown();
  return 0;
}
