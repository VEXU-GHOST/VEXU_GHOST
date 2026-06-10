#include "ghost_ros_interfaces/sensor_host/protocol.hpp"

namespace ghost_ros_interfaces::sensor_host
{

std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> & in)
{
  std::vector<uint8_t> out;
  out.reserve(in.size() + in.size() / 254 + 2);
  std::size_t code_pos = 0;
  out.push_back(0);          // placeholder for first code byte
  uint8_t code = 1;
  for (uint8_t b : in) {
    if (b != 0) {
      out.push_back(b);
      code++;
      if (code == 0xff) {
        out[code_pos] = code;
        code_pos = out.size();
        out.push_back(0);
        code = 1;
      }
    } else {
      out[code_pos] = code;
      code_pos = out.size();
      out.push_back(0);
      code = 1;
    }
  }
  out[code_pos] = code;
  return out;
}

std::vector<uint8_t> cobsDecode(const std::vector<uint8_t> & in)
{
  std::vector<uint8_t> out;
  std::size_t i = 0;
  uint8_t code = 0xff;
  uint8_t block = 0;
  while (i < in.size()) {
    if (block) {
      out.push_back(in[i++]);
      block--;
    } else {
      block = in[i++];
      if (block && code != 0xff) {
        out.push_back(0);
      }
      code = block;
      if (code == 0) {
        break;
      }
      block--;
    }
  }
  return out;
}

static uint8_t checksum(uint8_t cmd, const std::vector<uint8_t> & payload)
{
  uint8_t len_lo = static_cast<uint8_t>(payload.size() & 0xff);
  uint8_t len_hi = static_cast<uint8_t>((payload.size() >> 8) & 0xff);
  uint8_t cs = cmd + len_lo + len_hi;
  for (uint8_t b : payload) cs += b;
  return cs;
}

std::vector<uint8_t> encodeFrame(uint8_t cmd, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> body;
  body.reserve(payload.size() + 4);
  body.push_back(cmd);
  body.push_back(static_cast<uint8_t>(payload.size() & 0xff));
  body.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xff));
  body.insert(body.end(), payload.begin(), payload.end());
  body.push_back(checksum(cmd, payload));

  std::vector<uint8_t> wire = cobsEncode(body);
  wire.push_back(0x00);
  return wire;
}

bool decodeFrame(const std::vector<uint8_t> & cobs_chunk, uint8_t & cmd,
                 std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> body = cobsDecode(cobs_chunk);
  if (body.size() < 4) {
    return false;
  }
  uint16_t plen = body[1] | (static_cast<uint16_t>(body[2]) << 8);
  if (body.size() != static_cast<std::size_t>(3 + plen + 1)) {
    return false;
  }
  payload.assign(body.begin() + 3, body.begin() + 3 + plen);
  if (checksum(body[0], payload) != body[3 + plen]) {
    return false;
  }
  cmd = body[0];
  return true;
}

std::vector<uint8_t> buildI2CWrite(uint8_t port, uint8_t addr,
                                   const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> payload;
  payload.reserve(2 + data.size());
  payload.push_back(port);
  payload.push_back(addr);
  payload.insert(payload.end(), data.begin(), data.end());
  return encodeFrame(CMD_I2C_WRITE, payload);
}

std::vector<uint8_t> buildReadRequest(uint16_t id, uint8_t port, uint8_t addr,
                                      uint16_t interval_ms, uint16_t count,
                                      uint8_t read_len,
                                      const std::vector<uint8_t> & write_bytes,
                                      const std::vector<uint8_t> & post_bytes)
{
  std::vector<uint8_t> payload;
  payload.reserve(11 + write_bytes.size() + post_bytes.size());
  payload.push_back(id & 0xff);
  payload.push_back(id >> 8);
  payload.push_back(port);
  payload.push_back(addr);
  payload.push_back(interval_ms & 0xff);
  payload.push_back(interval_ms >> 8);
  payload.push_back(count & 0xff);
  payload.push_back(count >> 8);
  payload.push_back(read_len);
  payload.push_back(static_cast<uint8_t>(write_bytes.size()));
  payload.insert(payload.end(), write_bytes.begin(), write_bytes.end());
  payload.push_back(static_cast<uint8_t>(post_bytes.size()));
  payload.insert(payload.end(), post_bytes.begin(), post_bytes.end());
  return encodeFrame(CMD_READ_REQUEST, payload);
}

bool parseReadResult(const std::vector<uint8_t> & payload, ReadResult & out)
{
  if (payload.size() < 6) {
    return false;
  }
  out.id = payload[0] | (static_cast<uint16_t>(payload[1]) << 8);
  out.seq = payload[2] | (static_cast<uint16_t>(payload[3]) << 8);
  out.status = payload[4];
  uint8_t data_len = payload[5];
  if (payload.size() != static_cast<std::size_t>(6 + data_len)) {
    return false;
  }
  out.data.assign(payload.begin() + 6, payload.begin() + 6 + data_len);
  return true;
}

bool parseAck(const std::vector<uint8_t> & payload, Ack & out)
{
  if (payload.size() != 2) {
    return false;
  }
  out.ref_cmd = payload[0];
  out.status = payload[1];
  return true;
}

bool decodeIsl29125Rgb(const std::vector<uint8_t> & data, Rgb & out)
{
  if (data.size() < 6) {
    return false;
  }
  out.g = static_cast<uint16_t>(data[0] | (data[1] << 8));
  out.r = static_cast<uint16_t>(data[2] | (data[3] << 8));
  out.b = static_cast<uint16_t>(data[4] | (data[5] << 8));
  return true;
}

bool decodeVl53l4cdResult(const std::vector<uint8_t> & data, DistanceResult & out)
{
  if (data.size() < 15) {
    return false;
  }
  // ST GetResult() range-status remap (0x1F-masked raw -> human status).
  static const uint8_t status_rtn[24] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
    255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
    255, 255, 11, 12};
  uint8_t raw = data[0] & 0x1F;
  out.range_status = (raw < 24) ? status_rtn[raw] : 255;

  auto be16 = [&](std::size_t i) {
    return static_cast<uint16_t>((data[i] << 8) | data[i + 1]);
  };
  uint16_t signal = be16(5);    // RESULT__SIGNAL_RATE   (0x008E)
  uint16_t ambient = be16(7);   // RESULT__AMBIENT_RATE  (0x0090)
  uint16_t sigma = be16(9);     // RESULT__SIGMA         (0x0092)
  out.signal_rate_kcps = static_cast<uint32_t>(signal) * 8;
  out.ambient_rate_kcps = static_cast<uint32_t>(ambient) * 8;
  out.sigma_mm = sigma / 4;
  out.distance_mm = be16(13);   // RESULT__DISTANCE      (0x0096)
  return true;
}

std::vector<Frame> FrameAccumulator::feed(const uint8_t * data, std::size_t n)
{
  std::vector<Frame> frames;
  for (std::size_t i = 0; i < n; i++) {
    uint8_t b = data[i];
    if (b != 0x00) {
      buf_.push_back(b);
      continue;
    }
    if (!buf_.empty()) {
      Frame f;
      if (decodeFrame(buf_, f.cmd, f.payload)) {
        frames.push_back(std::move(f));
      }
      buf_.clear();
    }
  }
  return frames;
}

}  // namespace ghost_ros_interfaces::sensor_host
