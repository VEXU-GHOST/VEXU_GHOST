/*
Original IO expander driver code by leafony: https://github.com/leafony/arduino-tca9536
*/

#include "TCA9536.h"
#include <stdio.h>

TCA9536::TCA9536(void) {
  _i2c = nullptr;
  _deviceAddress = TCA9536_ADDRESS_INVALID;
}

TCA9536_error_t TCA9536::TCA9536_init(I2CBus* i2c, uint8_t address)
{
  // _deviceAddress = TCA9536_ADDRESS;
  _i2c = i2c;
  _deviceAddress = address;
  uint8_t buffer = 0;
  if (readI2CRegister(&buffer, TCA9536_REGISTER_CONFIGURATION) != TCA9536_ERROR_SUCCESS) {
    return TCA9536_ERROR_READ;
  }
  if (buffer != 0xFF) {
    // If the configuration register doesn't read as all inputs, it's possible the device is in a bad state. Try resetting it.
    if (writeI2CRegister(0xFF, TCA9536_REGISTER_CONFIGURATION) != TCA9536_ERROR_SUCCESS) {
      return TCA9536_ERROR_WRITE;
    }
  }
  return TCA9536_ERROR_SUCCESS;
  // return (isConnected());
}

TCA9536_error_t TCA9536::pinMode(uint8_t pin, uint8_t mode)
{
  TCA9536_error_t err;
  uint8_t cfgRegister = 0;

  if (pin > TCA9536_MAX_GPIO) return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&cfgRegister, TCA9536_REGISTER_CONFIGURATION);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  cfgRegister &= ~(1 << pin); // Clear pin bit
  if (mode == 1)          // Set the bit if it's being set to INPUT (opposite of Arduino)
  {
    cfgRegister |= (1 << pin);
  }
  return writeI2CRegister(cfgRegister, TCA9536_REGISTER_CONFIGURATION);
}

uint8_t TCA9536::getPinMode()
{
  TCA9536_error_t err;
  uint8_t cfgRegister = 0;

  err = readI2CRegister(&cfgRegister, TCA9536_REGISTER_CONFIGURATION);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  return cfgRegister & 0x0F; // Return the state of the 4 pins
}

TCA9536_error_t TCA9536::write(uint8_t pin, uint8_t value)
{
  TCA9536_error_t err;
  uint8_t outputRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&outputRegister, TCA9536_REGISTER_OUTPUT_PORT);
  printf("Current output register state: %02X\n", outputRegister);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  // TODO: Break out of here if it's already set correctly
  outputRegister &= ~(1 << pin); // Clear pin bit
  if (value == 1)             // Set the bit if it's being set to HIGH (opposite of Arduino)
  {
    outputRegister |= (1 << pin);
  }
  printf("New output register state: %02X\n", outputRegister);
  return writeI2CRegister(outputRegister, TCA9536_REGISTER_OUTPUT_PORT);
}

TCA9536_error_t TCA9536::digitalWrite(uint8_t pin, uint8_t value)
{
  return write(pin, value);
}

uint8_t TCA9536::readReg()
{
  TCA9536_error_t err;
  uint8_t inputRegister = 0;

  err = readI2CRegister(&inputRegister, TCA9536_REGISTER_INPUT_PORT);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  return (inputRegister & (0x0f));
}

uint8_t TCA9536::read(uint8_t pin)
{
  TCA9536_error_t err;
  uint8_t inputRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&inputRegister, TCA9536_REGISTER_INPUT_PORT);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  return (inputRegister & (1 << pin)) >> pin;
}

uint8_t TCA9536::digitalRead(uint8_t pin)
{
  return read(pin);
}

TCA9536_error_t TCA9536::invert(uint8_t pin, TCA9536_invert_t inversion)
{
  TCA9536_error_t err;
  uint8_t invertRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&invertRegister, TCA9536_REGISTER_POLARITY_INVERSION);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  // TODO: Break out of here if it's already set correctly
  invertRegister &= ~(1 << pin);   // Clear pin bit
  if (inversion == TCA9536_INVERT) // Set the bit if it's being set to inverted
  {
    invertRegister |= (1 << pin);
  }
  return writeI2CRegister(invertRegister, TCA9536_REGISTER_POLARITY_INVERSION);
}

TCA9536_error_t TCA9536::revert(uint8_t pin)
{
  return invert(pin, TCA9536_RETAIN);
}

TCA9536_error_t TCA9536::disablePullUp(bool flag)
{
  TCA9536_error_t err;
  uint8_t spfRegister = 0;

  err = readI2CRegister(&spfRegister, TCA9536_REGISTER_SPECIAL_FUNCTION);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  if (flag)
  {
    spfRegister |= 0x40; // Set bit
  }
  else
  {
    spfRegister &= 0xBF; // clear bit
  }
  return writeI2CRegister(spfRegister, TCA9536_REGISTER_SPECIAL_FUNCTION);
}

TCA9536_error_t TCA9536::readI2CBuffer(uint8_t *dest, TCA9536_REGISTER_t startRegister, uint16_t len)
{
  if (_deviceAddress == TCA9536_ADDRESS_INVALID)
  {
    return TCA9536_ERROR_INVALID_ADDRESS;
  }

  uint8_t start = (uint8_t)startRegister;
  int8_t ret_write = _i2c->write(_deviceAddress, &start, 1, true);

  if (ret_write == -1) {
    return TCA9536_ERROR_READ;
  }

  int8_t ret_read = _i2c->read(_deviceAddress, dest, len, false);

  if (ret_read == -1) {
    return TCA9536_ERROR_READ;
  }
  return TCA9536_ERROR_SUCCESS;
}

TCA9536_error_t TCA9536::writeI2CBuffer(uint8_t *src, TCA9536_REGISTER_t startRegister, uint16_t len)
{
  if (_deviceAddress == TCA9536_ADDRESS_INVALID)
  {
    return TCA9536_ERROR_INVALID_ADDRESS;
  }
  // Send register address and data in one transaction so the device sees
  // START + ADDR+W + REG + DATA + STOP (not two separate transactions).
  uint8_t buf[1 + len];
  buf[0] = (uint8_t)startRegister;
  for (uint16_t i = 0; i < len; i++) buf[1 + i] = src[i];
  int8_t ret_write = _i2c->write(_deviceAddress, buf, 1 + len, false);
  if (ret_write == -1) {
    return TCA9536_ERROR_WRITE;
  }
  return TCA9536_ERROR_SUCCESS;
}

TCA9536_error_t TCA9536::readI2CRegister(uint8_t *dest, TCA9536_REGISTER_t registerAddress)
{
  return readI2CBuffer(dest, registerAddress, 1);
}

TCA9536_error_t TCA9536::writeI2CRegister(uint8_t data, TCA9536_REGISTER_t registerAddress)
{
  return writeI2CBuffer(&data, registerAddress, 1);
}
