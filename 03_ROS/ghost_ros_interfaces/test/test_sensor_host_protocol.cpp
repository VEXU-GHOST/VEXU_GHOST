// Hardware-free unit tests for the sensor host wire-format codec.

#include <gtest/gtest.h>

#include "ghost_ros_interfaces/sensor_host/protocol.hpp"

namespace sh = ghost_ros_interfaces::sensor_host;

TEST(Cobs, RoundTripVariousBuffers)
{
  std::vector<std::vector<uint8_t>> cases = {
    {},
    {0x00},
    {0x01, 0x02, 0x03},
    {0x00, 0x00, 0x00},
    {0x11, 0x00, 0x22, 0x00, 0x33},
    std::vector<uint8_t>(600, 0xAB),   // forces a >254 COBS block
  };
  for (const auto & c : cases) {
    auto encoded = sh::cobsEncode(c);
    // COBS output must contain no zero bytes (so 0x00 can delimit frames).
    for (uint8_t b : encoded) {
      EXPECT_NE(b, 0x00);
    }
    EXPECT_EQ(sh::cobsDecode(encoded), c);
  }
}

TEST(Frame, EncodeDecodeRoundTrip)
{
  std::vector<uint8_t> payload = {0xDE, 0xAD, 0x00, 0xBE, 0xEF};
  auto wire = sh::encodeFrame(sh::CMD_READ_RESULT, payload);
  ASSERT_FALSE(wire.empty());
  EXPECT_EQ(wire.back(), 0x00);   // trailing delimiter

  std::vector<uint8_t> chunk(wire.begin(), wire.end() - 1);   // drop delimiter
  uint8_t cmd;
  std::vector<uint8_t> out;
  ASSERT_TRUE(sh::decodeFrame(chunk, cmd, out));
  EXPECT_EQ(cmd, sh::CMD_READ_RESULT);
  EXPECT_EQ(out, payload);
}

TEST(Frame, BadChecksumRejected)
{
  auto wire = sh::encodeFrame(sh::CMD_ACK, {0x10, 0x00});
  std::vector<uint8_t> chunk(wire.begin(), wire.end() - 1);
  std::vector<uint8_t> body = sh::cobsDecode(chunk);
  body.back() ^= 0xFF;                       // corrupt checksum
  auto bad = sh::cobsEncode(body);
  uint8_t cmd;
  std::vector<uint8_t> out;
  EXPECT_FALSE(sh::decodeFrame(bad, cmd, out));
}

TEST(Build, I2CWriteLayout)
{
  auto wire = sh::buildI2CWrite(6, 0x3b, {0x01, 0x05});
  std::vector<uint8_t> chunk(wire.begin(), wire.end() - 1);
  uint8_t cmd;
  std::vector<uint8_t> p;
  ASSERT_TRUE(sh::decodeFrame(chunk, cmd, p));
  EXPECT_EQ(cmd, sh::CMD_I2C_WRITE);
  EXPECT_EQ(p, (std::vector<uint8_t>{6, 0x3b, 0x01, 0x05}));
}

TEST(Build, ReadRequestLayout)
{
  auto wire = sh::buildReadRequest(0xA13F, 6, 0x3b, 100, 20, 6, {0x09});
  std::vector<uint8_t> chunk(wire.begin(), wire.end() - 1);
  uint8_t cmd;
  std::vector<uint8_t> p;
  ASSERT_TRUE(sh::decodeFrame(chunk, cmd, p));
  EXPECT_EQ(cmd, sh::CMD_READ_REQUEST);
  // id(2) port addr interval(2) count(2) read_len write_len write_bytes post_len
  EXPECT_EQ(p, (std::vector<uint8_t>{
    0x3F, 0xA1, 6, 0x3b, 100, 0, 20, 0, 6, 1, 0x09, 0}));
}

TEST(Build, ReadRequestWithPostWrite)
{
  // VL53L4CD-style: 16-bit pointer + post-write interrupt clear.
  auto wire = sh::buildReadRequest(0xD157, 6, 0x56, 100, 20, 15,
    {0x00, 0x89}, {0x00, 0x86, 0x01});
  std::vector<uint8_t> chunk(wire.begin(), wire.end() - 1);
  uint8_t cmd;
  std::vector<uint8_t> p;
  ASSERT_TRUE(sh::decodeFrame(chunk, cmd, p));
  EXPECT_EQ(cmd, sh::CMD_READ_REQUEST);
  EXPECT_EQ(p, (std::vector<uint8_t>{
    0x57, 0xD1, 6, 0x56, 100, 0, 20, 0, 15,
    2, 0x00, 0x89,           // write_len + pointer
    3, 0x00, 0x86, 0x01}));  // post_len + clear
}

TEST(Parse, ReadResult)
{
  std::vector<uint8_t> payload = {0x3F, 0xA1, 0x05, 0x00, sh::ST_OK, 0x06,
    0xFD, 0xFF, 0xFC, 0xFF, 0xFC, 0xFF};
  sh::ReadResult r;
  ASSERT_TRUE(sh::parseReadResult(payload, r));
  EXPECT_EQ(r.id, 0xA13F);
  EXPECT_EQ(r.seq, 5);
  EXPECT_EQ(r.status, sh::ST_OK);
  ASSERT_EQ(r.data.size(), 6u);
}

TEST(Parse, ReadResultLengthMismatch)
{
  // data_len says 6 but only 2 bytes present
  std::vector<uint8_t> payload = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x11, 0x22};
  sh::ReadResult r;
  EXPECT_FALSE(sh::parseReadResult(payload, r));
}

TEST(Decode, Isl29125Rgb)
{
  // [G_L G_H R_L R_H B_L B_H]
  std::vector<uint8_t> data = {0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A};
  sh::Rgb rgb;
  ASSERT_TRUE(sh::decodeIsl29125Rgb(data, rgb));
  EXPECT_EQ(rgb.g, 0x1234);
  EXPECT_EQ(rgb.r, 0x5678);
  EXPECT_EQ(rgb.b, 0x9ABC);
}

TEST(Decode, Vl53l4cdResult)
{
  // 15-byte block from RESULT__RANGE_STATUS (0x0089), big-endian words.
  std::vector<uint8_t> data = {
    0x09,        // range_status raw (status_rtn[9] = 0 valid)
    0x00, 0x00,  // 0x008A,0x008B
    0x00, 0x00,  // spad (0x008C)
    0x00, 0x64,  // signal 100 -> *8 = 800
    0x00, 0x0A,  // ambient 10 -> *8 = 80
    0x00, 0x10,  // sigma 16 -> /4 = 4
    0x00, 0x00,  // 0x0094,0x0095
    0x01, 0xF4,  // distance 500 mm
  };
  sh::DistanceResult d;
  ASSERT_TRUE(sh::decodeVl53l4cdResult(data, d));
  EXPECT_EQ(d.range_status, 0);
  EXPECT_EQ(d.distance_mm, 500);
  EXPECT_EQ(d.sigma_mm, 4);
  EXPECT_EQ(d.signal_rate_kcps, 800u);
  EXPECT_EQ(d.ambient_rate_kcps, 80u);
}

TEST(Decode, Vl53l4cdRangeStatusRemap)
{
  std::vector<uint8_t> data(15, 0);
  data[0] = 0x06;   // status_rtn[6] = 1
  sh::DistanceResult d;
  ASSERT_TRUE(sh::decodeVl53l4cdResult(data, d));
  EXPECT_EQ(d.range_status, 1);
}

TEST(Decode, Vl53l4cdTooShort)
{
  std::vector<uint8_t> data(10, 0);
  sh::DistanceResult d;
  EXPECT_FALSE(sh::decodeVl53l4cdResult(data, d));
}

TEST(Accumulator, SplitsStreamAndDropsGarbage)
{
  auto f1 = sh::encodeFrame(sh::CMD_ACK, {sh::CMD_I2C_WRITE, sh::ST_OK});
  auto f2 = sh::encodeFrame(sh::CMD_READ_RESULT,
    {0xDE, 0xC0, 0x00, 0x00, sh::ST_OK, 0x01, 0x42});

  std::vector<uint8_t> stream;
  stream.push_back(0x00);                                  // leading delimiter
  stream.insert(stream.end(), f1.begin(), f1.end());
  stream.push_back(0x00);                                  // extra delimiter (empty frame)
  stream.insert(stream.end(), f2.begin(), f2.end());

  sh::FrameAccumulator acc;
  // Feed in two arbitrary chunks to exercise buffering across reads.
  std::vector<sh::Frame> frames;
  std::size_t mid = stream.size() / 2;
  for (auto & f : acc.feed(stream.data(), mid)) frames.push_back(f);
  for (auto & f : acc.feed(stream.data() + mid, stream.size() - mid)) frames.push_back(f);

  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].cmd, sh::CMD_ACK);
  EXPECT_EQ(frames[1].cmd, sh::CMD_READ_RESULT);

  sh::ReadResult r;
  ASSERT_TRUE(sh::parseReadResult(frames[1].payload, r));
  EXPECT_EQ(r.id, 0xC0DE);
  EXPECT_EQ(r.data, (std::vector<uint8_t>{0x42}));
}
