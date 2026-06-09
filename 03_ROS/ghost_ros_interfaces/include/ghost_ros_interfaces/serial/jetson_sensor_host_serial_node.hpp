#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <ghost_msgs/msg/color_sensor_state.hpp>
#include <ghost_msgs/msg/distance_sensor_state.hpp>

#include "ghost_ros_interfaces/sensor_host/protocol.hpp"

namespace ghost_ros_interfaces
{

// Drives the sensor host (a generic I2C bridge) over USB serial.
//
// Devices are configured once with a synchronous handshake, then each is left
// running as an autonomous recurring read on the host: the host writes a
// register pointer, reads, and (for sensors that need it) writes a post-read
// "clear" to advance to the next sample — all without per-sample round-trips to
// the Orin, which just receives the streamed results. See PROTOCOL.md.
class JetsonSensorHostSerialNode : public rclcpp::Node
{
public:
  JetsonSensorHostSerialNode();
  ~JetsonSensorHostSerialNode();

private:
  bool openSerial();
  void writeFrame(const std::vector<uint8_t> & wire);

  // Synchronous request/response helpers, used only during one-time device init
  // (before the async read thread starts).
  bool readResponse(uint8_t want_cmd, std::vector<uint8_t> & payload, int timeout_ms);
  bool syncWrite(uint8_t port, uint8_t addr, const std::vector<uint8_t> & bytes);
  bool syncRead(uint8_t port, uint8_t addr, const std::vector<uint8_t> & pointer,
                uint8_t len, std::vector<uint8_t> & out);
  bool syncWriteReg16(uint8_t port, uint8_t addr, uint16_t reg,
                      const std::vector<uint8_t> & data);
  bool syncReadReg16(uint8_t port, uint8_t addr, uint16_t reg, uint8_t len,
                     std::vector<uint8_t> & out);

  // Colour sensor (ISL29125)
  bool initColor();
  void sendColorReadRequest();

  // Distance sensor (VL53L4CD)
  bool vl53l4cdInit();
  bool vl53l4cdWaitDataReady(uint8_t port, uint8_t addr);
  bool vl53l4cdSetRangeTiming(uint8_t port, uint8_t addr, uint32_t timing_budget_ms);
  void sendDistanceReadRequest();

  void readLoop();
  void handleColorResult(const sensor_host::ReadResult & result);
  void handleDistanceResult(const sensor_host::ReadResult & result);

  // Parameters
  std::string serial_port_;
  bool color_enabled_;
  int color_port_;
  int color_addr_;
  bool distance_enabled_;
  int distance_port_;
  int distance_addr_;
  int read_count_;
  int interval_ms_;
  uint16_t color_read_id_;
  uint16_t distance_read_id_;

  int fd_;
  std::atomic_bool running_;
  std::thread read_thread_;
  sensor_host::FrameAccumulator accumulator_;       // async streaming
  sensor_host::FrameAccumulator init_accumulator_;  // synchronous init

  rclcpp::Publisher<ghost_msgs::msg::ColorSensorState>::SharedPtr color_pub_;
  rclcpp::Publisher<ghost_msgs::msg::DistanceSensorState>::SharedPtr distance_pub_;
  rclcpp::TimerBase::SharedPtr rearm_timer_;
};

}  // namespace ghost_ros_interfaces
