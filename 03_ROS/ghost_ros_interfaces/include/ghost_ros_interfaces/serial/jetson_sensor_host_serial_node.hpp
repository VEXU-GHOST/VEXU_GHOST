#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ghost_msgs/msg/color_sensor_state.hpp>
#include <ghost_msgs/msg/distance_sensor_state.hpp>

#include "ghost_ros_interfaces/sensor_host/protocol.hpp"

namespace ghost_ros_interfaces
{

// One configured sensor on the host (see ghost_sensor_host/PROTOCOL.md §8).
struct SensorDevice
{
  std::string name;
  std::string type;        // COLOR | DISTANCE | IMU | IO_EXPANDER
  uint8_t port = 0;
  uint8_t addr = 0;
  uint16_t read_id = 0;
  uint16_t interval_ms = 100;
  uint16_t read_count = 20;

  // type-specific config (defaults applied at load)
  int config1 = 0x0D;          // COLOR: ISL29125 CONFIG1
  int config2 = 0xBF;          // COLOR: ISL29125 CONFIG2
  int timing_budget_ms = 50;   // DISTANCE: VL53L4CD timing budget

  rclcpp::Publisher<ghost_msgs::msg::ColorSensorState>::SharedPtr color_pub;
  rclcpp::Publisher<ghost_msgs::msg::DistanceSensorState>::SharedPtr distance_pub;
};

// Drives the sensor host (a generic I2C bridge) over USB serial. Configured by a
// device map (loaded from a yaml-cpp file): each device is initialised once with
// a synchronous handshake, then left running as an autonomous recurring read on
// the host, and published on /<namespace>/<type>/<name>.
class JetsonSensorHostSerialNode : public rclcpp::Node
{
public:
  JetsonSensorHostSerialNode();
  ~JetsonSensorHostSerialNode();

private:
  bool openSerial();
  void writeFrame(const std::vector<uint8_t> & wire);

  // Synchronous request/response helpers (one-time device init only).
  bool readResponse(uint8_t want_cmd, std::vector<uint8_t> & payload, int timeout_ms);
  bool syncWrite(uint8_t port, uint8_t addr, const std::vector<uint8_t> & bytes);
  bool syncRead(uint8_t port, uint8_t addr, const std::vector<uint8_t> & pointer,
                uint8_t len, std::vector<uint8_t> & out);
  bool syncWriteReg16(uint8_t port, uint8_t addr, uint16_t reg,
                      const std::vector<uint8_t> & data);
  bool syncReadReg16(uint8_t port, uint8_t addr, uint16_t reg, uint8_t len,
                     std::vector<uint8_t> & out);

  // Device registry
  bool loadDevices(const std::string & path);
  bool initDevice(SensorDevice & dev);
  void sendReadRequest(const SensorDevice & dev);
  void handleResult(const SensorDevice & dev, const sensor_host::ReadResult & result);
  void readLoop();

  // COLOR (ISL29125)
  bool initColor(const SensorDevice & dev);
  void publishColor(const SensorDevice & dev, const sensor_host::ReadResult & result);

  // DISTANCE (VL53L4CD)
  bool vl53l4cdInit(const SensorDevice & dev);
  bool vl53l4cdWaitDataReady(uint8_t port, uint8_t addr);
  bool vl53l4cdSetRangeTiming(uint8_t port, uint8_t addr, uint32_t timing_budget_ms);
  void publishDistance(const SensorDevice & dev, const sensor_host::ReadResult & result);

  // Parameters
  std::string serial_port_;
  std::string topic_namespace_;

  std::vector<SensorDevice> devices_;
  std::unordered_map<uint16_t, std::size_t> id_to_device_;

  int fd_;
  std::atomic_bool running_;
  std::thread read_thread_;
  sensor_host::FrameAccumulator accumulator_;       // async streaming
  sensor_host::FrameAccumulator init_accumulator_;  // synchronous init
  rclcpp::TimerBase::SharedPtr rearm_timer_;
};

}  // namespace ghost_ros_interfaces
