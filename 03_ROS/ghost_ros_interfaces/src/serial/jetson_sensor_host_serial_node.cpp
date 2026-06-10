#include "ghost_ros_interfaces/serial/jetson_sensor_host_serial_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using ghost_ros_interfaces::sensor_host::DistanceResult;
using ghost_ros_interfaces::sensor_host::ReadResult;
using ghost_ros_interfaces::sensor_host::Rgb;

namespace ghost_ros_interfaces
{

namespace
{
constexpr uint16_t SYNC_READ_ID = 0x0001;   // id for one-time init reads
constexpr uint16_t FIRST_DEVICE_ID = 0x0100;

// Rotary DAC offset XOR'd onto a device's default address per switch position.
const uint8_t DAC_OFFSET[16] = {
  0x7F, 0x75, 0x7A, 0x70, 0x2F, 0x25, 0x2A, 0x20,
  0x4F, 0x45, 0x4A, 0x40, 0x0F, 0x05, 0x0A, 0x00};

int default_addr_for_type(const std::string & t)
{
  if (t == "COLOR") return 0x44;        // ISL29125
  if (t == "DISTANCE") return 0x29;     // VL53L4CD
  if (t == "IMU") return 0x68;          // ICM20602
  if (t == "IO_EXPANDER") return 0x41;  // TCA9536
  return -1;
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

// Read a YAML scalar as int, tolerating hex ("0x0D") and decimal.
int yint(const YAML::Node & n, int dflt)
{
  if (!n) return dflt;
  try {
    return static_cast<int>(std::stol(n.as<std::string>(), nullptr, 0));
  } catch (...) {
    return dflt;
  }
}
}  // namespace

// ISL29125 colour sensor (8-bit registers).
namespace isl29125
{
constexpr uint8_t CONFIG1 = 0x01;
constexpr uint8_t CONFIG2 = 0x02;
constexpr uint8_t CONFIG3 = 0x03;
constexpr uint8_t GREEN_DATA_L = 0x09;
constexpr uint8_t COLOR_DATA_LEN = 6;
}  // namespace isl29125

// VL53L4CD distance sensor (16-bit registers, big-endian). Mirrors the ST API.
namespace vl53
{
constexpr uint16_t IDENTIFICATION_MODEL_ID = 0x010F;   // expect 0xEBAA
constexpr uint16_t FIRMWARE_SYSTEM_STATUS = 0x00E5;    // boot done == 0x03
constexpr uint16_t GPIO_HV_MUX_CTRL = 0x0030;
constexpr uint16_t GPIO_TIO_HV_STATUS = 0x0031;
constexpr uint16_t SYSTEM_INTERRUPT_CLEAR = 0x0086;
constexpr uint16_t SYSTEM_START = 0x0087;
constexpr uint16_t VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
constexpr uint16_t INTERMEASUREMENT_MS = 0x006C;
constexpr uint16_t RANGE_CONFIG_A = 0x005E;
constexpr uint16_t RANGE_CONFIG_B = 0x0061;
constexpr uint16_t OSC_FREQUENCY = 0x0006;
constexpr uint16_t CONFIG_START = 0x002D;
constexpr uint16_t RESULT_RANGE_STATUS = 0x0089;
constexpr uint8_t RESULT_BLOCK_LEN = 15;

const std::vector<uint8_t> DEFAULT_CONFIG = {
  0x00,
  0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10,
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x14,
  0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00,
  0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xcc,
  0x07, 0x01, 0xf1, 0x05, 0x00, 0xa0, 0x00, 0x80, 0x08, 0x38,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00, 0x00,
  0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
};
}  // namespace vl53

JetsonSensorHostSerialNode::JetsonSensorHostSerialNode()
: Node("ghost_sensor_host_serial_node"),
  fd_(-1),
  running_(false)
{
  serial_port_     = declare_parameter("serial_port", "/dev/ttyACM0");
  topic_namespace_ = declare_parameter("topic_namespace", "/sensors");
  std::string device_config = declare_parameter("device_config", "");

  if (device_config.empty() || !loadDevices(device_config)) {
    RCLCPP_ERROR(get_logger(), "No devices configured (device_config='%s')", device_config.c_str());
    return;
  }

  if (!openSerial()) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Opened %s; %zu device(s)", serial_port_.c_str(), devices_.size());

  // Let the USB CDC connection settle; the first bytes after open can be dropped.
  std::this_thread::sleep_for(300ms);
  tcflush(fd_, TCIOFLUSH);

  // ---- One-time synchronous init + publisher per device --------------------
  for (std::size_t i = 0; i < devices_.size(); i++) {
    SensorDevice & dev = devices_[i];
    std::string topic = topic_namespace_ + "/" + lower(dev.type) + "/" + dev.name;
    if (dev.type == "COLOR") {
      dev.color_pub = create_publisher<ghost_msgs::msg::ColorSensorState>(
        topic, rclcpp::SensorDataQoS());
    } else if (dev.type == "DISTANCE") {
      dev.distance_pub = create_publisher<ghost_msgs::msg::DistanceSensorState>(
        topic, rclcpp::SensorDataQoS());
    }
    bool ok = initDevice(dev);
    RCLCPP_INFO(get_logger(), "%s '%s' port=%d addr=0x%02x -> %s : init %s",
      dev.type.c_str(), dev.name.c_str(), dev.port, dev.addr, topic.c_str(),
      ok ? "OK" : "FAILED");
    id_to_device_[dev.read_id] = i;
  }
  tcflush(fd_, TCIOFLUSH);

  // ---- Start autonomous recurring reads, stream results --------------------
  running_ = true;
  read_thread_ = std::thread(&JetsonSensorHostSerialNode::readLoop, this);
  for (const auto & dev : devices_) {
    sendReadRequest(dev);
  }
  // Re-arm the finite recurring reads before the shortest-lived one expires.
  uint32_t rearm_ms = 2000;
  for (const auto & dev : devices_) {
    rearm_ms = std::min<uint32_t>(rearm_ms, uint32_t(dev.read_count) * dev.interval_ms);
  }
  if (rearm_ms == 0) rearm_ms = 2000;
  rearm_timer_ = create_wall_timer(
    std::chrono::milliseconds(rearm_ms),
    [this]() {
      for (const auto & dev : devices_) sendReadRequest(dev);
    });
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

bool JetsonSensorHostSerialNode::loadDevices(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Cannot load device_config '%s': %s", path.c_str(), e.what());
    return false;
  }
  YAML::Node devs = root["devices"];
  if (!devs || !devs.IsMap()) {
    RCLCPP_ERROR(get_logger(), "device_config '%s' has no 'devices' map", path.c_str());
    return false;
  }

  uint16_t next_id = FIRST_DEVICE_ID;
  for (const auto & it : devs) {
    SensorDevice dev;
    dev.name = it.first.as<std::string>();
    YAML::Node d = it.second;

    dev.type = d["type"] ? d["type"].as<std::string>() : "";
    std::transform(dev.type.begin(), dev.type.end(), dev.type.begin(),
      [](unsigned char c) { return std::toupper(c); });

    int def = default_addr_for_type(dev.type);
    if (def < 0) {
      RCLCPP_WARN(get_logger(), "device '%s': unknown type '%s', skipping",
        dev.name.c_str(), dev.type.c_str());
      continue;
    }
    int port = yint(d["port"], -1);
    int sw = yint(d["remap_switch"], 0);
    if (port < 0 || port > 7 || sw < 0 || sw > 15) {
      RCLCPP_WARN(get_logger(), "device '%s': bad port/remap_switch, skipping", dev.name.c_str());
      continue;
    }
    int addr = def ^ DAC_OFFSET[sw];
    if (addr < 0x08 || addr > 0x77) {
      RCLCPP_WARN(get_logger(), "device '%s': remap_switch %d gives reserved addr 0x%02x, skipping",
        dev.name.c_str(), sw, addr);
      continue;
    }
    dev.port = static_cast<uint8_t>(port);
    dev.addr = static_cast<uint8_t>(addr);
    dev.interval_ms = static_cast<uint16_t>(yint(d["interval_ms"], 100));
    dev.read_count = static_cast<uint16_t>(yint(d["read_count"], 20));
    dev.config1 = yint(d["config1"], 0x0D);
    dev.config2 = yint(d["config2"], 0xBF);
    dev.timing_budget_ms = yint(d["timing_budget_ms"], 50);
    dev.read_id = next_id++;
    devices_.push_back(std::move(dev));
  }
  if (devices_.empty()) {
    RCLCPP_ERROR(get_logger(), "device_config '%s' produced no usable devices", path.c_str());
    return false;
  }
  return true;
}

// COLOR and DISTANCE are implemented. To add IMU or IO_EXPANDER, port the
// device's I2C init/read sequence here (the way initColor / vl53l4cdInit do).
// The original on-host firmware drivers were deleted from polling_firmware but
// remain in git history as a register/sequence reference:
//   IMU         -> 02_V5/.../polling_firmware/src/ICM20602/*  (ICM20602)
//   IO_EXPANDER -> 02_V5/.../polling_firmware/src/TCA9536.cpp  (TCA9536)
// (git log --follow / git show <rev>:<path> on those paths.)
bool JetsonSensorHostSerialNode::initDevice(SensorDevice & dev)
{
  if (dev.type == "COLOR") return initColor(dev);
  if (dev.type == "DISTANCE") return vl53l4cdInit(dev);
  RCLCPP_WARN(get_logger(), "type '%s' not implemented yet (no streaming)", dev.type.c_str());
  return false;
}

void JetsonSensorHostSerialNode::sendReadRequest(const SensorDevice & dev)
{
  if (dev.type == "COLOR") {
    writeFrame(sensor_host::buildReadRequest(
      dev.read_id, dev.port, dev.addr, dev.interval_ms, dev.read_count,
      isl29125::COLOR_DATA_LEN, {isl29125::GREEN_DATA_L}));
  } else if (dev.type == "DISTANCE") {
    writeFrame(sensor_host::buildReadRequest(
      dev.read_id, dev.port, dev.addr, dev.interval_ms, dev.read_count,
      vl53::RESULT_BLOCK_LEN,
      {uint8_t(vl53::RESULT_RANGE_STATUS >> 8), uint8_t(vl53::RESULT_RANGE_STATUS & 0xFF)},
      {uint8_t(vl53::SYSTEM_INTERRUPT_CLEAR >> 8), uint8_t(vl53::SYSTEM_INTERRUPT_CLEAR & 0xFF), 0x01}));
  }
}

void JetsonSensorHostSerialNode::handleResult(
  const SensorDevice & dev, const ReadResult & result)
{
  if (result.status != sensor_host::ST_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "%s '%s' read error status=%d", dev.type.c_str(), dev.name.c_str(), result.status);
    return;
  }
  if (dev.type == "COLOR") {
    publishColor(dev, result);
  } else if (dev.type == "DISTANCE") {
    publishDistance(dev, result);
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
  tty.c_cc[VTIME] = 1;
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
  if (fd_ >= 0) {
    ssize_t n = ::write(fd_, wire.data(), wire.size());
    if (n != static_cast<ssize_t>(wire.size())) {
      RCLCPP_WARN(get_logger(), "Short serial write (%zd/%zu)", n, wire.size());
    }
  }
}

bool JetsonSensorHostSerialNode::readResponse(
  uint8_t want_cmd, std::vector<uint8_t> & payload, int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  uint8_t buf[256];
  while (std::chrono::steady_clock::now() < deadline) {
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }
    for (auto & frame : init_accumulator_.feed(buf, static_cast<std::size_t>(n))) {
      if (frame.cmd == want_cmd) {
        payload = frame.payload;
        return true;
      }
    }
  }
  return false;
}

bool JetsonSensorHostSerialNode::syncWrite(
  uint8_t port, uint8_t addr, const std::vector<uint8_t> & bytes)
{
  writeFrame(sensor_host::buildI2CWrite(port, addr, bytes));
  std::vector<uint8_t> ack;
  if (!readResponse(sensor_host::CMD_ACK, ack, 200)) {
    return false;
  }
  sensor_host::Ack a;
  return sensor_host::parseAck(ack, a) && a.status == sensor_host::ST_OK;
}

bool JetsonSensorHostSerialNode::syncRead(
  uint8_t port, uint8_t addr, const std::vector<uint8_t> & pointer, uint8_t len,
  std::vector<uint8_t> & out)
{
  writeFrame(sensor_host::buildReadRequest(SYNC_READ_ID, port, addr, 0, 1, len, pointer));
  std::vector<uint8_t> p;
  if (!readResponse(sensor_host::CMD_READ_RESULT, p, 300)) {
    return false;
  }
  ReadResult r;
  if (!sensor_host::parseReadResult(p, r) || r.id != SYNC_READ_ID || r.status != sensor_host::ST_OK) {
    return false;
  }
  out = r.data;
  return out.size() >= len;
}

bool JetsonSensorHostSerialNode::syncWriteReg16(
  uint8_t port, uint8_t addr, uint16_t reg, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> bytes = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
  bytes.insert(bytes.end(), data.begin(), data.end());
  return syncWrite(port, addr, bytes);
}

bool JetsonSensorHostSerialNode::syncReadReg16(
  uint8_t port, uint8_t addr, uint16_t reg, uint8_t len, std::vector<uint8_t> & out)
{
  return syncRead(port, addr,
    {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)}, len, out);
}

// ---- Colour sensor (ISL29125) ---------------------------------------------

bool JetsonSensorHostSerialNode::initColor(const SensorDevice & dev)
{
  bool ok = true;
  ok &= syncWrite(dev.port, dev.addr, {isl29125::CONFIG1, static_cast<uint8_t>(dev.config1)});
  ok &= syncWrite(dev.port, dev.addr, {isl29125::CONFIG2, static_cast<uint8_t>(dev.config2)});
  ok &= syncWrite(dev.port, dev.addr, {isl29125::CONFIG3, 0x00});
  return ok;
}

void JetsonSensorHostSerialNode::publishColor(
  const SensorDevice & dev, const ReadResult & result)
{
  Rgb rgb;
  if (!sensor_host::decodeIsl29125Rgb(result.data, rgb)) {
    return;
  }
  ghost_msgs::msg::ColorSensorState msg;
  msg.name = dev.name;
  msg.r = rgb.r;
  msg.g = rgb.g;
  msg.b = rgb.b;
  msg.lux = 0;
  msg.cct = 0;
  dev.color_pub->publish(msg);
}

// ---- Distance sensor (VL53L4CD) -------------------------------------------

bool JetsonSensorHostSerialNode::vl53l4cdWaitDataReady(uint8_t port, uint8_t addr)
{
  std::vector<uint8_t> buf;
  for (int i = 0; i < 1000; i++) {
    if (!syncReadReg16(port, addr, vl53::GPIO_HV_MUX_CTRL, 1, buf)) {
      return false;
    }
    uint8_t int_pol = (((buf[0] & 0x10) >> 4) == 1) ? 0 : 1;
    if (!syncReadReg16(port, addr, vl53::GPIO_TIO_HV_STATUS, 1, buf)) {
      return false;
    }
    if ((buf[0] & 0x01) == int_pol) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

bool JetsonSensorHostSerialNode::vl53l4cdSetRangeTiming(
  uint8_t port, uint8_t addr, uint32_t timing_budget_ms)
{
  std::vector<uint8_t> buf;
  if (!syncReadReg16(port, addr, vl53::OSC_FREQUENCY, 2, buf)) {
    return false;
  }
  uint16_t osc = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  if (osc == 0) {
    return false;
  }
  uint32_t timing_budget_us = timing_budget_ms * 1000u;
  uint32_t macro_period_us = static_cast<uint32_t>(2304u * (0x40000000u / osc)) >> 6;

  if (!syncWriteReg16(port, addr, vl53::INTERMEASUREMENT_MS, {0, 0, 0, 0})) {
    return false;
  }
  timing_budget_us -= 2500u;
  uint32_t tb = timing_budget_us << 12;

  auto config_word = [&](uint32_t macro_mult) {
    uint16_t ms_byte = 0;
    uint32_t tmp = macro_period_us * macro_mult;
    uint32_t ls_byte = ((tb + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1u;
    while ((ls_byte & 0xFFFFFF00u) > 0u) {
      ls_byte >>= 1;
      ms_byte++;
    }
    return static_cast<uint16_t>((ms_byte << 8) + (ls_byte & 0xFF));
  };

  uint16_t cfg_a = config_word(16);
  if (!syncWriteReg16(port, addr, vl53::RANGE_CONFIG_A,
    {static_cast<uint8_t>(cfg_a >> 8), static_cast<uint8_t>(cfg_a & 0xFF)})) {
    return false;
  }
  uint16_t cfg_b = config_word(12);
  return syncWriteReg16(port, addr, vl53::RANGE_CONFIG_B,
    {static_cast<uint8_t>(cfg_b >> 8), static_cast<uint8_t>(cfg_b & 0xFF)});
}

bool JetsonSensorHostSerialNode::vl53l4cdInit(const SensorDevice & dev)
{
  const uint8_t port = dev.port;
  const uint8_t addr = dev.addr;
  std::vector<uint8_t> buf;

  if (!syncReadReg16(port, addr, vl53::IDENTIFICATION_MODEL_ID, 2, buf)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD '%s': no response reading model id", dev.name.c_str());
    return false;
  }
  uint16_t model_id = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  if (model_id != 0xEBAA) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD '%s': unexpected model id 0x%04x", dev.name.c_str(), model_id);
    return false;
  }

  bool booted = false;
  for (int i = 0; i < 1000 && !booted; i++) {
    if (syncReadReg16(port, addr, vl53::FIRMWARE_SYSTEM_STATUS, 1, buf) && buf[0] == 0x03) {
      booted = true;
    } else {
      std::this_thread::sleep_for(1ms);
    }
  }
  if (!booted) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD '%s': firmware boot timeout", dev.name.c_str());
    return false;
  }

  if (!syncWriteReg16(port, addr, vl53::CONFIG_START, vl53::DEFAULT_CONFIG)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD '%s': config blob write failed", dev.name.c_str());
    return false;
  }

  syncWriteReg16(port, addr, vl53::SYSTEM_START, {0x40});
  if (!vl53l4cdWaitDataReady(port, addr)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD '%s': VHV data-ready timeout", dev.name.c_str());
    return false;
  }
  syncWriteReg16(port, addr, vl53::SYSTEM_INTERRUPT_CLEAR, {0x01});
  syncWriteReg16(port, addr, vl53::SYSTEM_START, {0x80});
  syncWriteReg16(port, addr, vl53::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, {0x09});
  syncWriteReg16(port, addr, 0x000B, {0x00});
  syncWriteReg16(port, addr, 0x0024, {0x05, 0x00});

  if (!vl53l4cdSetRangeTiming(port, addr, static_cast<uint32_t>(dev.timing_budget_ms))) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD '%s': SetRangeTiming failed", dev.name.c_str());
    return false;
  }

  syncWriteReg16(port, addr, vl53::SYSTEM_START, {0x21});
  syncWriteReg16(port, addr, vl53::SYSTEM_INTERRUPT_CLEAR, {0x01});
  return true;
}

void JetsonSensorHostSerialNode::publishDistance(
  const SensorDevice & dev, const ReadResult & result)
{
  DistanceResult d;
  if (!sensor_host::decodeVl53l4cdResult(result.data, d)) {
    return;
  }
  ghost_msgs::msg::DistanceSensorState msg;
  msg.name = dev.name;
  msg.distance_mm = d.distance_mm;
  msg.range_status = d.range_status;
  msg.sigma_mm = d.sigma_mm;
  msg.signal_rate_kcps = d.signal_rate_kcps;
  msg.ambient_rate_kcps = d.ambient_rate_kcps;
  dev.distance_pub->publish(msg);
}

// ---- Streaming read loop --------------------------------------------------

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
      if (!sensor_host::parseReadResult(frame.payload, result)) {
        continue;
      }
      auto it = id_to_device_.find(result.id);
      if (it != id_to_device_.end()) {
        handleResult(devices_[it->second], result);
      }
    }
  }
}

}  // namespace ghost_ros_interfaces

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_ros_interfaces::JetsonSensorHostSerialNode>());
  rclcpp::shutdown();
  return 0;
}
