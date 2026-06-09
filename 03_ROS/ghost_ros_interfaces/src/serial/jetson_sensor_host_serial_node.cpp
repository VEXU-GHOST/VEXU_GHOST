/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.
 *
 *   MIT License (see repository).
 */

#include "ghost_ros_interfaces/serial/jetson_sensor_host_serial_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

using namespace std::chrono_literals;
using ghost_ros_interfaces::sensor_host::ReadResult;
using ghost_ros_interfaces::sensor_host::Rgb;

namespace ghost_ros_interfaces
{

// ISL29125 register map (device logic lives here on the ROS side).
namespace isl29125
{
constexpr uint8_t CONFIG1 = 0x01;
constexpr uint8_t CONFIG2 = 0x02;
constexpr uint8_t CONFIG3 = 0x03;
constexpr uint8_t GREEN_DATA_L = 0x09;   // first of 6 colour data registers
constexpr uint8_t CONFIG1_RGB_MODE = 0x05;
constexpr uint8_t COLOR_DATA_LEN = 6;
}  // namespace isl29125

JetsonSensorHostSerialNode::JetsonSensorHostSerialNode()
: Node("ghost_sensor_host_serial_node"),
  fd_(-1),
  running_(false)
{
  serial_port_   = declare_parameter("serial_port", "/dev/ttyACM0");
  color_port_    = declare_parameter("color_port", 6);          // input 7
  color_addr_    = declare_parameter("color_addr", 0x3b);       // rotary switch 0
  read_count_    = declare_parameter("read_count", 20);
  interval_ms_   = declare_parameter("interval_ms", 100);
  color_read_id_ = static_cast<uint16_t>(declare_parameter("color_read_id", 0xC0DE));

  color_pub_ = create_publisher<ghost_msgs::msg::ColorSensorState>(
    "sensor_host/color_sensor_update", rclcpp::SensorDataQoS());

  if (!openSerial()) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Opened %s; color sensor port=%d addr=0x%02x",
    serial_port_.c_str(), color_port_, color_addr_);

  running_ = true;
  read_thread_ = std::thread(&JetsonSensorHostSerialNode::readLoop, this);

  sendColorInit();
  sendColorReadRequest();

  // Re-arm the finite recurring read before it expires so the stream continues.
  rearm_timer_ = create_wall_timer(
    std::chrono::milliseconds(read_count_ * interval_ms_),
    [this]() { sendColorReadRequest(); });
}

JetsonSensorHostSerialNode::~JetsonSensorHostSerialNode()
{
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool JetsonSensorHostSerialNode::openSerial()
{
  fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    return false;
  }

  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;   // 0.1 s read timeout
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void JetsonSensorHostSerialNode::writeFrame(const std::vector<uint8_t> & wire)
{
  if (fd_ < 0) {
    return;
  }
  ssize_t n = ::write(fd_, wire.data(), wire.size());
  if (n != static_cast<ssize_t>(wire.size())) {
    RCLCPP_WARN(get_logger(), "Short serial write (%zd/%zu)", n, wire.size());
  }
}

void JetsonSensorHostSerialNode::sendColorInit()
{
  const uint8_t port = static_cast<uint8_t>(color_port_);
  const uint8_t addr = static_cast<uint8_t>(color_addr_);
  writeFrame(sensor_host::buildI2CWrite(port, addr, {isl29125::CONFIG1, isl29125::CONFIG1_RGB_MODE}));
  std::this_thread::sleep_for(10ms);
  writeFrame(sensor_host::buildI2CWrite(port, addr, {isl29125::CONFIG2, 0x00}));
  std::this_thread::sleep_for(10ms);
  writeFrame(sensor_host::buildI2CWrite(port, addr, {isl29125::CONFIG3, 0x00}));
  std::this_thread::sleep_for(10ms);
  RCLCPP_INFO(get_logger(), "Sent ISL29125 init writes");
}

void JetsonSensorHostSerialNode::sendColorReadRequest()
{
  writeFrame(sensor_host::buildReadRequest(
    color_read_id_,
    static_cast<uint8_t>(color_port_),
    static_cast<uint8_t>(color_addr_),
    static_cast<uint16_t>(interval_ms_),
    static_cast<uint16_t>(read_count_),
    isl29125::COLOR_DATA_LEN,
    {isl29125::GREEN_DATA_L}));
}

void JetsonSensorHostSerialNode::readLoop()
{
  uint8_t buf[256];
  while (running_) {
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }
    for (auto & frame : accumulator_.feed(buf, static_cast<std::size_t>(n))) {
      if (frame.cmd != sensor_host::CMD_READ_RESULT) {
        continue;
      }
      ReadResult result;
      if (sensor_host::parseReadResult(frame.payload, result)) {
        handleReadResult(result);
      }
    }
  }
}

void JetsonSensorHostSerialNode::handleReadResult(const ReadResult & result)
{
  if (result.id != color_read_id_) {
    return;
  }
  if (result.status != sensor_host::ST_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Color read error status=%d", result.status);
    return;
  }
  Rgb rgb;
  if (!sensor_host::decodeIsl29125Rgb(result.data, rgb)) {
    return;
  }
  ghost_msgs::msg::ColorSensorState msg;
  msg.name = "color_sensor";
  msg.r = rgb.r;
  msg.g = rgb.g;
  msg.b = rgb.b;
  msg.lux = 0;
  msg.cct = 0;
  color_pub_->publish(msg);
}

}  // namespace ghost_ros_interfaces

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_ros_interfaces::JetsonSensorHostSerialNode>());
  rclcpp::shutdown();
  return 0;
}
