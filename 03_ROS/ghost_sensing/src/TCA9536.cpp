#include "ghost_sensing/TCA9536.hpp"
#include "linux_i2c_interface.h"
#include <memory>

namespace ghost_sensing {

TCA9536::TCA9536(uint8_t I2c_address, std::vector<uint8_t> initial_input_states)
  : I2c_address(I2c_address), inputs(initial_input_states)
{
  // minimal change: create the linux i2c interface as the header expects a shared_ptr
  m_i2c_communication = std::make_shared<linux_i2c_interface>(std::string("/dev/i2c-1"), rclcpp::get_logger("TCA9536"));
  m_i2c_communication->init();

  // configure device; guard if initial_input_states shorter than 4
  uint8_t v0 = inputs.size() > 0 ? inputs[0] : 0;
  uint8_t v1 = inputs.size() > 1 ? inputs[1] : 0;
  uint8_t v2 = inputs.size() > 2 ? inputs[2] : 0;
  uint8_t v3 = inputs.size() > 3 ? inputs[3] : 0;

  write_register(0x03, v2);
  write_register(0x01, v0);
  write_register(0x02, v1);
  write_register(0x50, v3);
}

void TCA9536::write_register(uint8_t reg, uint8_t value)
{
  m_i2c_communication->write(I2c_address, reg, &value, 1);
}

uint8_t TCA9536::read_register(uint8_t reg)
{
  uint8_t value = 0;
  m_i2c_communication->read(I2c_address, reg, &value, 1);
  return value;
}

} // namespace ghost_sensing