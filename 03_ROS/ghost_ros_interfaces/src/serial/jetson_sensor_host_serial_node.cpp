#include "ghost_ros_interfaces/serial/jetson_sensor_host_serial_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

using namespace std::chrono_literals;
using ghost_ros_interfaces::sensor_host::DistanceResult;
using ghost_ros_interfaces::sensor_host::ReadResult;
using ghost_ros_interfaces::sensor_host::Rgb;

namespace ghost_ros_interfaces
{

namespace
{
constexpr uint16_t SYNC_READ_ID = 0x0001;   // id for one-time init reads
}  // namespace

// ISL29125 colour sensor (8-bit registers).
namespace isl29125
{
constexpr uint8_t CONFIG1 = 0x01;
constexpr uint8_t CONFIG2 = 0x02;
constexpr uint8_t CONFIG3 = 0x03;
constexpr uint8_t GREEN_DATA_L = 0x09;     // first of 6 colour data registers
constexpr uint8_t CONFIG1_RGB_MODE = 0x05;
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
constexpr uint16_t CONFIG_START = 0x002D;              // default config blob start
constexpr uint16_t RESULT_RANGE_STATUS = 0x0089;       // 15-byte result block
constexpr uint8_t RESULT_BLOCK_LEN = 15;

// Default configuration written to 0x2D..0x87 (91 bytes), I2C fast mode.
// Verbatim from ST's VL53L4CD_DEFAULT_CONFIGURATION.
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
  serial_port_      = declare_parameter("serial_port", "/dev/ttyACM0");
  color_enabled_    = declare_parameter("color_enabled", true);
  color_port_       = declare_parameter("color_port", 6);          // input 7
  color_addr_       = declare_parameter("color_addr", 0x3b);       // rotary switch 0
  // CONFIG1 0x0D = RGB mode (0x05) + 10,000-lux range (bit 3) + 16-bit.
  // Use 0x05 for the low 375-lux range, or set bit 4 for 12-bit.
  color_config1_    = declare_parameter("color_config1", 0x0D);
  color_config2_    = declare_parameter("color_config2", 0xBF);    // max IR compensation
  distance_enabled_ = declare_parameter("distance_enabled", true);
  distance_port_    = declare_parameter("distance_port", 6);       // input 7
  distance_addr_    = declare_parameter("distance_addr", 0x56);    // rotary switch 0
  read_count_       = declare_parameter("read_count", 20);
  interval_ms_      = declare_parameter("interval_ms", 100);
  color_read_id_    = static_cast<uint16_t>(declare_parameter("color_read_id", 0xC0DE));
  distance_read_id_ = static_cast<uint16_t>(declare_parameter("distance_read_id", 0xD157));

  color_pub_ = create_publisher<ghost_msgs::msg::ColorSensorState>(
    "sensor_host/color_sensor_update", rclcpp::SensorDataQoS());
  distance_pub_ = create_publisher<ghost_msgs::msg::DistanceSensorState>(
    "sensor_host/distance_sensor_update", rclcpp::SensorDataQoS());

  if (!openSerial()) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Opened %s", serial_port_.c_str());

  // Let the USB CDC connection settle; the first bytes after open can be dropped.
  std::this_thread::sleep_for(300ms);
  tcflush(fd_, TCIOFLUSH);

  // ---- One-time synchronous device init (async read thread not yet running) --
  if (color_enabled_) {
    RCLCPP_INFO(get_logger(), "ISL29125 init %s (port=%d addr=0x%02x)",
      initColor() ? "OK" : "FAILED", color_port_, color_addr_);
  }
  if (distance_enabled_) {
    RCLCPP_INFO(get_logger(), "VL53L4CD init %s (port=%d addr=0x%02x)",
      vl53l4cdInit() ? "OK" : "FAILED", distance_port_, distance_addr_);
  }
  tcflush(fd_, TCIOFLUSH);   // drop init ACKs / results

  // ---- Start autonomous recurring reads, stream results --------------------
  running_ = true;
  read_thread_ = std::thread(&JetsonSensorHostSerialNode::readLoop, this);
  if (color_enabled_) {
    sendColorReadRequest();
  }
  if (distance_enabled_) {
    sendDistanceReadRequest();
  }
  // Re-arm the finite recurring reads before they expire so streaming continues.
  rearm_timer_ = create_wall_timer(
    std::chrono::milliseconds(read_count_ * interval_ms_),
    [this]() {
      if (color_enabled_) sendColorReadRequest();
      if (distance_enabled_) sendDistanceReadRequest();
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

bool JetsonSensorHostSerialNode::initColor()
{
  const uint8_t port = static_cast<uint8_t>(color_port_);
  const uint8_t addr = static_cast<uint8_t>(color_addr_);
  bool ok = true;
  ok &= syncWrite(port, addr, {isl29125::CONFIG1, static_cast<uint8_t>(color_config1_)});
  ok &= syncWrite(port, addr, {isl29125::CONFIG2, static_cast<uint8_t>(color_config2_)});
  ok &= syncWrite(port, addr, {isl29125::CONFIG3, 0x00});
  return ok;
}

void JetsonSensorHostSerialNode::sendColorReadRequest()
{
  // Simple register device: no post-write needed.
  writeFrame(sensor_host::buildReadRequest(
    color_read_id_,
    static_cast<uint8_t>(color_port_),
    static_cast<uint8_t>(color_addr_),
    static_cast<uint16_t>(interval_ms_),
    static_cast<uint16_t>(read_count_),
    isl29125::COLOR_DATA_LEN,
    {isl29125::GREEN_DATA_L}));
}

void JetsonSensorHostSerialNode::handleColorResult(const ReadResult & result)
{
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

bool JetsonSensorHostSerialNode::vl53l4cdInit()
{
  const uint8_t port = static_cast<uint8_t>(distance_port_);
  const uint8_t addr = static_cast<uint8_t>(distance_addr_);
  std::vector<uint8_t> buf;

  if (!syncReadReg16(port, addr, vl53::IDENTIFICATION_MODEL_ID, 2, buf)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD: no response reading model id");
    return false;
  }
  uint16_t model_id = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  if (model_id != 0xEBAA) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD: unexpected model id 0x%04x", model_id);
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
    RCLCPP_ERROR(get_logger(), "VL53L4CD: firmware boot timeout");
    return false;
  }

  if (!syncWriteReg16(port, addr, vl53::CONFIG_START, vl53::DEFAULT_CONFIG)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD: config blob write failed");
    return false;
  }

  syncWriteReg16(port, addr, vl53::SYSTEM_START, {0x40});            // start VHV
  if (!vl53l4cdWaitDataReady(port, addr)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD: VHV data-ready timeout");
    return false;
  }
  syncWriteReg16(port, addr, vl53::SYSTEM_INTERRUPT_CLEAR, {0x01});
  syncWriteReg16(port, addr, vl53::SYSTEM_START, {0x80});            // stop ranging
  syncWriteReg16(port, addr, vl53::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, {0x09});
  syncWriteReg16(port, addr, 0x000B, {0x00});
  syncWriteReg16(port, addr, 0x0024, {0x05, 0x00});

  if (!vl53l4cdSetRangeTiming(port, addr, 50)) {
    RCLCPP_ERROR(get_logger(), "VL53L4CD: SetRangeTiming failed");
    return false;
  }

  syncWriteReg16(port, addr, vl53::SYSTEM_START, {0x21});            // continuous
  syncWriteReg16(port, addr, vl53::SYSTEM_INTERRUPT_CLEAR, {0x01});
  return true;
}

void JetsonSensorHostSerialNode::sendDistanceReadRequest()
{
  // Each iteration: write pointer -> read 15 result bytes -> write interrupt
  // clear (post). The host advances the measurement on its own.
  writeFrame(sensor_host::buildReadRequest(
    distance_read_id_,
    static_cast<uint8_t>(distance_port_),
    static_cast<uint8_t>(distance_addr_),
    static_cast<uint16_t>(interval_ms_),
    static_cast<uint16_t>(read_count_),
    vl53::RESULT_BLOCK_LEN,
    {static_cast<uint8_t>(vl53::RESULT_RANGE_STATUS >> 8),
      static_cast<uint8_t>(vl53::RESULT_RANGE_STATUS & 0xFF)},
    {static_cast<uint8_t>(vl53::SYSTEM_INTERRUPT_CLEAR >> 8),
      static_cast<uint8_t>(vl53::SYSTEM_INTERRUPT_CLEAR & 0xFF), 0x01}));
}

void JetsonSensorHostSerialNode::handleDistanceResult(const ReadResult & result)
{
  if (result.status != sensor_host::ST_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Distance read error status=%d", result.status);
    return;
  }
  DistanceResult d;
  if (!sensor_host::decodeVl53l4cdResult(result.data, d)) {
    return;
  }
  ghost_msgs::msg::DistanceSensorState msg;
  msg.name = "distance_sensor";
  msg.distance_mm = d.distance_mm;
  msg.range_status = d.range_status;
  msg.sigma_mm = d.sigma_mm;
  msg.signal_rate_kcps = d.signal_rate_kcps;
  msg.ambient_rate_kcps = d.ambient_rate_kcps;
  distance_pub_->publish(msg);
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
      if (result.id == color_read_id_) {
        handleColorResult(result);
      } else if (result.id == distance_read_id_) {
        handleDistanceResult(result);
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
