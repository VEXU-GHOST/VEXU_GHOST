#include "../common/vexlink_protocol.hpp"

#include <cassert>
#include <cstdint>

int main()
{
  const vexlink::Payload original{0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0xFF};
  vexlink::Payload decoded{};

  std::uint8_t radio_packet[vexlink::RADIO_PACKET_SIZE]{};
  assert(vexlink::encode_radio_payload(
      radio_packet, sizeof(radio_packet), original) == sizeof(radio_packet));
  assert(vexlink::decode_radio_payload(radio_packet, sizeof(radio_packet), decoded));
  assert(decoded == original);
  assert(!vexlink::decode_radio_payload(radio_packet, sizeof(radio_packet) - 1, decoded));

  radio_packet[0] = 0xFF;
  assert(!vexlink::decode_radio_payload(radio_packet, sizeof(radio_packet), decoded));
  radio_packet[0] = static_cast<std::uint8_t>(vexlink::MsgType::RADIO_PAYLOAD);
  radio_packet[1] = static_cast<std::uint8_t>(vexlink::PAYLOAD_SIZE - 1);
  assert(!vexlink::decode_radio_payload(radio_packet, sizeof(radio_packet), decoded));

  std::uint8_t serial_packet[vexlink::SERIAL_PAYLOAD_PACKET_SIZE]{};
  assert(vexlink::encode_serial_payload(
      serial_packet, sizeof(serial_packet), vexlink::MsgType::HOST_TO_RADIO,
      original) == sizeof(serial_packet));

  vexlink::HostSerialParser parser;
  vexlink::Payload received{};
  assert(!parser.feed('x', received));
  for (std::size_t i = 0; i < sizeof(serial_packet) - 1; ++i) {
    assert(!parser.feed(serial_packet[i], received));
  }
  assert(parser.feed(serial_packet[sizeof(serial_packet) - 1], received));
  assert(received == original);

  assert(vexlink::encode_serial_payload(
      serial_packet, sizeof(serial_packet), vexlink::MsgType::RADIO_TO_HOST,
      original) == sizeof(serial_packet));
  for (const std::uint8_t byte : serial_packet) {
    assert(!parser.feed(byte, received));
  }
  assert(parser.takeRejectedFrames() == 1);

  serial_packet[2] = static_cast<std::uint8_t>(vexlink::MsgType::HOST_TO_RADIO);
  serial_packet[3] = static_cast<std::uint8_t>(vexlink::PAYLOAD_SIZE + 1);
  for (const std::uint8_t byte : serial_packet) {
    assert(!parser.feed(byte, received));
  }
  assert(parser.takeRejectedFrames() == 1);

  assert(vexlink::encode_serial_payload(
      serial_packet, sizeof(serial_packet), vexlink::MsgType::HOST_TO_RADIO,
      original) == sizeof(serial_packet));
  bool parsed = false;
  for (const std::uint8_t byte : serial_packet) {
    parsed = parser.feed(byte, received) || parsed;
  }
  assert(parsed);
  assert(received == original);

  vexlink::PayloadQueue queue;
  for (std::size_t i = 0; i < vexlink::HOST_PAYLOAD_QUEUE_CAPACITY; ++i) {
    vexlink::Payload payload{};
    payload[0] = static_cast<std::uint8_t>(i);
    assert(queue.push(payload));
  }
  assert(!queue.push(original));
  for (std::size_t i = 0; i < vexlink::HOST_PAYLOAD_QUEUE_CAPACITY; ++i) {
    assert(queue.pop(decoded));
    assert(decoded[0] == i);
  }
  assert(!queue.pop(decoded));

  return 0;
}
