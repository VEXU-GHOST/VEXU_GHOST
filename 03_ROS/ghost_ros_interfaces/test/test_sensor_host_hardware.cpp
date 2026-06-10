// Hardware-in-the-loop test for the sensor host color sensor.
//
// Auto-skips when no sensor host is connected, so `colcon test` passes on a
// machine with no PCB. When the Pico bridge IS plugged in (and an ISL29125
// colour sensor wired to the configured port), it drives a real read and
// asserts a valid colour result comes back.
//
// Overridable via environment:
//   SENSOR_HOST_TTY   serial device   (default /dev/ttyACM0)
//   SENSOR_HOST_PORT  protocol port   (default 6  = input 7)
//   SENSOR_HOST_ADDR  I2C address     (default 0x3b)

#include <gtest/gtest.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "ghost_ros_interfaces/sensor_host/protocol.hpp"

namespace sh = ghost_ros_interfaces::sensor_host;

namespace
{
std::string env_or(const char * key, const std::string & dflt)
{
  const char * v = std::getenv(key);
  return v ? std::string(v) : dflt;
}
int env_int(const char * key, int dflt)
{
  const char * v = std::getenv(key);
  return v ? static_cast<int>(std::strtol(v, nullptr, 0)) : dflt;
}

int openSerial(const std::string & path)
{
  int fd = open(path.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    return -1;
  }
  struct termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return -1;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}
}  // namespace

TEST(SensorHostHardware, ReadsColorSensor)
{
  const std::string tty = env_or("SENSOR_HOST_TTY", "/dev/ttyACM0");
  const uint8_t port = static_cast<uint8_t>(env_int("SENSOR_HOST_PORT", 6));
  const uint8_t addr = static_cast<uint8_t>(env_int("SENSOR_HOST_ADDR", 0x3b));

  if (access(tty.c_str(), R_OK | W_OK) != 0) {
    GTEST_SKIP() << "No sensor host at " << tty << " — skipping hardware test.";
  }
  int fd = openSerial(tty);
  if (fd < 0) {
    GTEST_SKIP() << "Could not open " << tty << " — skipping hardware test.";
  }

  auto write_frame = [&](const std::vector<uint8_t> & w) {
    ASSERT_EQ(::write(fd, w.data(), w.size()), static_cast<ssize_t>(w.size()));
  };

  // Initialize the ISL29125 (CONFIG1 = RGB mode).
  write_frame(sh::buildI2CWrite(port, addr, {0x01, 0x05}));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  write_frame(sh::buildI2CWrite(port, addr, {0x02, 0x00}));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  write_frame(sh::buildI2CWrite(port, addr, {0x03, 0x00}));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Request a short recurring read of the colour registers.
  const uint16_t id = 0x7357;
  write_frame(sh::buildReadRequest(id, port, addr, 100, 10, 6, {0x09}));

  // Read for up to ~2.5 s, looking for a successful colour result.
  sh::FrameAccumulator acc;
  bool got_ok = false;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
  uint8_t buf[256];
  while (!got_ok && std::chrono::steady_clock::now() < deadline) {
    ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }
    for (auto & frame : acc.feed(buf, static_cast<std::size_t>(n))) {
      if (frame.cmd != sh::CMD_READ_RESULT) {
        continue;
      }
      sh::ReadResult r;
      if (sh::parseReadResult(frame.payload, r) && r.id == id && r.status == sh::ST_OK) {
        sh::Rgb rgb;
        ASSERT_TRUE(sh::decodeIsl29125Rgb(r.data, rgb));
        RecordProperty("r", rgb.r);
        RecordProperty("g", rgb.g);
        RecordProperty("b", rgb.b);
        got_ok = true;
        break;
      }
    }
  }
  close(fd);

  EXPECT_TRUE(got_ok) << "No successful color READ_RESULT received from " << tty
                      << " (port " << int(port) << ", addr 0x" << std::hex << int(addr)
                      << "). Is the ISL29125 wired to that input?";
}
