#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <linux_i2c_interface.h>

namespace ghost_sensing {

class TCA9536 {
public:

    TCA9536(uint8_t I2c_address, std::vector<uint8_t> initial_input_states);
    void write_register(uint8_t reg, uint8_t value);
    uint8_t read_register(uint8_t reg);
private:
    std::shared_ptr<linux_i2c_interface> m_i2c_communication;
    uint8_t I2c_address;
    std::vector<uint8_t> inputs;
};
}