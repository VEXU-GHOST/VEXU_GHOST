#include "PCF8575.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>

bool PCF8575::open(const std::string & i2cBus, uint8_t address, uint16_t initVal)
{
	printf("YO\n");
  _address = address;

	printf("YO2\n");
  _fd = ::open(i2cBus.c_str(), O_RDWR);
  if (_fd < 0) {
    perror("PCF8575: Failed to open I2C bus");
	return false;
  }
	printf("YO3\n");
  if (ioctl(_fd, I2C_SLAVE, _address) < 0) {
    perror("PCF8575: Failed to set I2C address");
	return false;
  }
  current_out = initVal;
  return true;
}


PCF8575::~PCF8575()
{
  if (_fd >= 0) {
    close(_fd);
  }
}

uint16_t PCF8575::read16()
{
  uint8_t data[2];
  int ret = ::read(_fd, data, 2);
  if (ret != 2) {
    perror("PCF8575: Failed to read 2 bytes");
    return 0;
  }
  // Combine low and high byte into a 16-bit value.
  return data[0] | (data[1] << 8);
}

bool PCF8575::write16(uint16_t value)
{
  uint8_t data[2];
  data[0] = value & 0xFF;            // low byte
  data[1] = (value >> 8) & 0xFF;       // high byte
  // TODO: ^^ is so fucking dumb, chatgpt wrote it, todo fix using cast assignment or smth
  int ret = ::write(_fd, data, 2);
  if (ret != 2) {
    perror("PCF8575: Failed to write 2 bytes");
    return false;
  }
  return true;
}

bool PCF8575::setPin(uint8_t pin, bool value)
{
  if (pin > 15) {return false;}
  if (value) {
    current_out |= (1 << pin);
  } else {
    current_out &= ~(1 << pin);
  }
  return true;
}

bool PCF8575::getPin(uint8_t pin, bool & value)
{
  if (pin > 15) {return false;}
  value = (current_in >> pin) & 0x01;
  return true;
}

bool PCF8575::poll()
{
  current_in = read16();
  return write16(current_out);
}
