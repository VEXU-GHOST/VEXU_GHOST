#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace vexlink
{

constexpr std::size_t PAYLOAD_SIZE = 7;
using Payload = std::array<std::uint8_t, PAYLOAD_SIZE>;

enum class MsgType : std::uint8_t
{
  IMU_DATA = 0x03,
  RADIO_PAYLOAD = 0x10,
  HOST_TO_RADIO = 0x11,
  RADIO_TO_HOST = 0x12,
};

struct __attribute__((packed)) ImuMsg
{
  float heading;
  float pitch;
  float roll;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accel_x;
  float accel_y;
  float accel_z;
};

constexpr std::uint8_t RADIO_HEADER_SIZE = 2;
constexpr std::size_t RADIO_PACKET_SIZE = RADIO_HEADER_SIZE + PAYLOAD_SIZE;
constexpr std::uint8_t SERIAL_SYNC_0 = 0xAA;
constexpr std::uint8_t SERIAL_SYNC_1 = 0xBB;
constexpr std::uint8_t SERIAL_HEADER_SIZE = 4;
constexpr std::size_t SERIAL_PAYLOAD_PACKET_SIZE = SERIAL_HEADER_SIZE + PAYLOAD_SIZE;
constexpr std::size_t SERIAL_MAX_PACKET_SIZE =
  SERIAL_HEADER_SIZE + sizeof(ImuMsg);
constexpr std::size_t HOST_PAYLOAD_QUEUE_CAPACITY = 8;

static_assert(sizeof(ImuMsg) == 36, "IMU serial payload must remain 36 bytes");

inline std::size_t encode_radio_payload(
  std::uint8_t * buffer, std::size_t buffer_size, const Payload & payload)
{
  if (buffer == nullptr || buffer_size < RADIO_PACKET_SIZE) {
    return 0;
  }

  buffer[0] = static_cast<std::uint8_t>(MsgType::RADIO_PAYLOAD);
  buffer[1] = static_cast<std::uint8_t>(PAYLOAD_SIZE);
  std::memcpy(buffer + RADIO_HEADER_SIZE, payload.data(), PAYLOAD_SIZE);
  return RADIO_PACKET_SIZE;
}

inline bool decode_radio_payload(
  const std::uint8_t * buffer, std::size_t buffer_size, Payload & payload)
{
  if (buffer == nullptr || buffer_size != RADIO_PACKET_SIZE ||
    buffer[0] != static_cast<std::uint8_t>(MsgType::RADIO_PAYLOAD) ||
    buffer[1] != PAYLOAD_SIZE)
  {
    return false;
  }

  std::memcpy(payload.data(), buffer + RADIO_HEADER_SIZE, PAYLOAD_SIZE);
  return true;
}

inline std::size_t encode_serial_bytes(
  std::uint8_t * buffer, std::size_t buffer_size, MsgType type,
  const std::uint8_t * payload, std::size_t payload_size)
{
  if (buffer == nullptr || payload == nullptr || payload_size > UINT8_MAX ||
    buffer_size < SERIAL_HEADER_SIZE + payload_size)
  {
    return 0;
  }

  buffer[0] = SERIAL_SYNC_0;
  buffer[1] = SERIAL_SYNC_1;
  buffer[2] = static_cast<std::uint8_t>(type);
  buffer[3] = static_cast<std::uint8_t>(payload_size);
  std::memcpy(buffer + SERIAL_HEADER_SIZE, payload, payload_size);
  return SERIAL_HEADER_SIZE + payload_size;
}

template<typename T>
inline std::size_t encode_serial(
  std::uint8_t * buffer, std::size_t buffer_size, MsgType type, const T & payload)
{
  static_assert(std::is_trivially_copyable<T>::value, "Serial payload must be trivially copyable");
  constexpr std::size_t payload_size = sizeof(T);
  return encode_serial_bytes(
    buffer, buffer_size, type,
    reinterpret_cast<const std::uint8_t *>(&payload), payload_size);
}

inline std::size_t encode_serial_payload(
  std::uint8_t * buffer, std::size_t buffer_size, MsgType type, const Payload & payload)
{
  if (type != MsgType::HOST_TO_RADIO && type != MsgType::RADIO_TO_HOST) {
    return 0;
  }
  return encode_serial_bytes(
    buffer, buffer_size, type, payload.data(), payload.size());
}

class HostSerialParser
{
public:
  bool feed(std::uint8_t byte, Payload & payload)
  {
    if (buffered_ == 0) {
      if (byte == SERIAL_SYNC_0) {
        buffer_[buffered_++] = byte;
      }
      return false;
    }

    if (buffered_ == 1) {
      if (byte == SERIAL_SYNC_1) {
        buffer_[buffered_++] = byte;
      } else if (byte != SERIAL_SYNC_0) {
        buffered_ = 0;
      }
      return false;
    }

    buffer_[buffered_++] = byte;
    if (buffered_ == SERIAL_HEADER_SIZE &&
      (buffer_[2] != static_cast<std::uint8_t>(MsgType::HOST_TO_RADIO) ||
      buffer_[3] != PAYLOAD_SIZE))
    {
      ++rejected_frames_;
      resetFrom(byte);
      return false;
    }

    if (buffered_ == SERIAL_PAYLOAD_PACKET_SIZE) {
      std::memcpy(payload.data(), buffer_.data() + SERIAL_HEADER_SIZE, PAYLOAD_SIZE);
      buffered_ = 0;
      return true;
    }

    return false;
  }

  std::uint32_t takeRejectedFrames()
  {
    const std::uint32_t rejected = rejected_frames_;
    rejected_frames_ = 0;
    return rejected;
  }

private:
  void resetFrom(std::uint8_t byte)
  {
    buffered_ = byte == SERIAL_SYNC_0 ? 1 : 0;
    if (buffered_ == 1) {
      buffer_[0] = byte;
    }
  }

  std::array<std::uint8_t, SERIAL_PAYLOAD_PACKET_SIZE> buffer_{};
  std::size_t buffered_ = 0;
  std::uint32_t rejected_frames_ = 0;
};

class PayloadQueue
{
public:
  bool push(const Payload & payload)
  {
    if (size_ == queue_.size()) {
      return false;
    }

    queue_[(head_ + size_) % queue_.size()] = payload;
    ++size_;
    return true;
  }

  bool front(Payload & payload) const
  {
    if (size_ == 0) {
      return false;
    }

    payload = queue_[head_];
    return true;
  }

  bool discardFront()
  {
    if (size_ == 0) {
      return false;
    }

    head_ = (head_ + 1) % queue_.size();
    --size_;
    return true;
  }

  bool pop(Payload & payload)
  {
    if (!front(payload)) {
      return false;
    }
    return discardFront();
  }

  std::size_t size() const
  {
    return size_;
  }

private:
  std::array<Payload, HOST_PAYLOAD_QUEUE_CAPACITY> queue_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

} // namespace vexlink
