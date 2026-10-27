#pragma once

#include "vexlink_protocol.hpp"

#include <algorithm>
#include <cstdio>
#include <unistd.h>

#include "pros/apix.h"
#include "pros/link.hpp"

namespace vexlink::pros_transport
{

constexpr std::uint32_t MANAGER_TX_INTERVAL_MS = 20;
constexpr std::uint32_t WORKER_TX_INTERVAL_MS = 40;
constexpr std::uint32_t LOOP_INTERVAL_MS = 5;
constexpr std::size_t SERIAL_READ_CHUNK_SIZE = 64;

struct HostReadResult
{
  std::uint32_t frames_enqueued = 0;
  std::uint32_t frames_dropped = 0;
  std::uint32_t frames_rejected = 0;
  bool io_error = false;
};

enum class RadioReceiveResult
{
  EMPTY,
  RECEIVED,
  INVALID,
  IO_ERROR,
};

inline bool intervalElapsed(
  std::uint32_t now, std::uint32_t & last, std::uint32_t interval_ms)
{
  if (now - last < interval_ms) {
    return false;
  }
  last = now;
  return true;
}

inline HostReadResult readHostSerial(
  int input_fd, HostSerialParser & parser, PayloadQueue & queue)
{
  HostReadResult result;
  const std::int32_t available = pros::c::fdctl(input_fd, DEVCTL_FIONREAD, nullptr);
  if (available == PROS_ERR || available < 0) {
    result.io_error = true;
    return result;
  }
  if (available == 0) {
    return result;
  }

  std::uint8_t read_buffer[SERIAL_READ_CHUNK_SIZE];
  const std::size_t read_size = std::min<std::size_t>(
    static_cast<std::size_t>(available), sizeof(read_buffer));
  const ssize_t bytes_read = read(input_fd, read_buffer, read_size);
  if (bytes_read < 0) {
    result.io_error = true;
    return result;
  }

  Payload payload{};
  for (ssize_t i = 0; i < bytes_read; ++i) {
    if (parser.feed(read_buffer[i], payload)) {
      if (queue.push(payload)) {
        ++result.frames_enqueued;
      } else {
        ++result.frames_dropped;
      }
    }
  }
  result.frames_rejected = parser.takeRejectedFrames();
  return result;
}

inline bool writeSerialFrame(
  std::uint8_t * buffer, std::size_t buffer_size, std::size_t packet_size)
{
  if (packet_size == 0 || packet_size > buffer_size) {
    return false;
  }
  const std::size_t bytes_written = std::fwrite(buffer, 1, packet_size, stdout);
  const int flush_result = std::fflush(stdout);
  return bytes_written == packet_size && flush_result == 0;
}

inline bool writePayloadToHost(const Payload & payload)
{
  std::uint8_t buffer[SERIAL_PAYLOAD_PACKET_SIZE];
  const std::size_t packet_size = encode_serial_payload(
    buffer, sizeof(buffer), MsgType::RADIO_TO_HOST, payload);
  return writeSerialFrame(buffer, sizeof(buffer), packet_size);
}

inline bool sendRadioPayload(pros::Link & link, const Payload & payload)
{
  std::uint8_t buffer[RADIO_PACKET_SIZE];
  const std::size_t packet_size = encode_radio_payload(
    buffer, sizeof(buffer), payload);
  if (packet_size != sizeof(buffer)) {
    return false;
  }

  return link.transmit_raw(buffer, static_cast<std::uint16_t>(packet_size)) ==
    packet_size;
}

inline RadioReceiveResult receiveRadioPayload(pros::Link & link, Payload & payload)
{
  const std::uint32_t available = link.raw_receivable_size();
  if (available == static_cast<std::uint32_t>(PROS_ERR)) {
    return RadioReceiveResult::IO_ERROR;
  }
  if (available < RADIO_PACKET_SIZE) {
    return RadioReceiveResult::EMPTY;
  }

  std::uint8_t buffer[RADIO_PACKET_SIZE];
  const std::uint32_t bytes_read = link.receive_raw(
    buffer, static_cast<std::uint16_t>(sizeof(buffer)));
  if (bytes_read == static_cast<std::uint32_t>(PROS_ERR) ||
    bytes_read != sizeof(buffer))
  {
    return RadioReceiveResult::IO_ERROR;
  }
  return decode_radio_payload(buffer, bytes_read, payload) ?
    RadioReceiveResult::RECEIVED : RadioReceiveResult::INVALID;
}

} // namespace vexlink::pros_transport
