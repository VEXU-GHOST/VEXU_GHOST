/*
 * Sensor Host serial protocol codec (see ghost_sensor_host/PROTOCOL.md).
 *
 * Pure, I/O-free encode/decode helpers so the wire format can be unit-tested
 * without any hardware. The ROS node layers serial I/O on top of these.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace ghost_ros_interfaces::sensor_host
{

// Command ids
constexpr uint8_t CMD_ACK          = 0x01;
constexpr uint8_t CMD_I2C_WRITE    = 0x10;
constexpr uint8_t CMD_READ_REQUEST = 0x11;
constexpr uint8_t CMD_READ_RESULT  = 0x12;

// Status codes
constexpr uint8_t ST_OK          = 0;
constexpr uint8_t ST_BAD_PARAMS  = 1;
constexpr uint8_t ST_WRITE_FAIL  = 2;
constexpr uint8_t ST_READ_FAIL   = 3;
constexpr uint8_t ST_TABLE_FULL  = 4;

// ---- COBS ----------------------------------------------------------------
std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> & in);
std::vector<uint8_t> cobsDecode(const std::vector<uint8_t> & in);

// ---- Framing -------------------------------------------------------------
// Wire frame: COBS([cmd][len_lo][len_hi][payload][checksum]) + 0x00.
// `encodeFrame` returns the full wire bytes including the trailing 0x00.
std::vector<uint8_t> encodeFrame(uint8_t cmd, const std::vector<uint8_t> & payload);

// Decode one COBS chunk (the bytes between delimiters, no 0x00). Returns true
// and fills cmd/payload if length and checksum are valid.
bool decodeFrame(const std::vector<uint8_t> & cobs_chunk, uint8_t & cmd,
                 std::vector<uint8_t> & payload);

// ---- Command builders ----------------------------------------------------
std::vector<uint8_t> buildI2CWrite(uint8_t port, uint8_t addr,
                                   const std::vector<uint8_t> & data);

// `write_bytes` are written before each read (register pointer); `post_bytes`
// are written after each read (e.g. clearing a data-ready interrupt).
std::vector<uint8_t> buildReadRequest(uint16_t id, uint8_t port, uint8_t addr,
                                      uint16_t interval_ms, uint16_t count,
                                      uint8_t read_len,
                                      const std::vector<uint8_t> & write_bytes,
                                      const std::vector<uint8_t> & post_bytes = {});

// ---- Response parsing ----------------------------------------------------
struct ReadResult
{
  uint16_t id;
  uint16_t seq;
  uint8_t status;
  std::vector<uint8_t> data;
};
bool parseReadResult(const std::vector<uint8_t> & payload, ReadResult & out);

struct Ack
{
  uint8_t ref_cmd;
  uint8_t status;
};
bool parseAck(const std::vector<uint8_t> & payload, Ack & out);

// ---- Device decode (ISL29125 colour sensor) ------------------------------
struct Rgb
{
  uint16_t r;
  uint16_t g;
  uint16_t b;
};
// Decodes a 6-byte read of registers 0x09..0x0E: [G_L G_H R_L R_H B_L B_H].
bool decodeIsl29125Rgb(const std::vector<uint8_t> & data, Rgb & out);

// ---- Device decode (VL53L4CD distance sensor) ----------------------------
struct DistanceResult
{
  uint16_t distance_mm;
  uint8_t range_status;       // 0 = valid measurement
  uint16_t sigma_mm;
  uint32_t signal_rate_kcps;
  uint32_t ambient_rate_kcps;
};
// Decodes a 15-byte block read starting at RESULT__RANGE_STATUS (0x0089), the
// big-endian result registers 0x0089..0x0097 (range_status, spad, signal,
// ambient, sigma, distance). Mirrors the ST driver's GetResult().
bool decodeVl53l4cdResult(const std::vector<uint8_t> & data, DistanceResult & out);

// ---- Streaming frame accumulator -----------------------------------------
struct Frame
{
  uint8_t cmd;
  std::vector<uint8_t> payload;
};

// Feed raw serial bytes; returns each complete, valid frame as it completes.
// Garbage between frames and bad checksums are silently dropped.
class FrameAccumulator
{
public:
  std::vector<Frame> feed(const uint8_t * data, std::size_t n);

private:
  std::vector<uint8_t> buf_;
};

}  // namespace ghost_ros_interfaces::sensor_host
