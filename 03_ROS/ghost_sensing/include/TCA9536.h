#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include "driver_tcs34725_interface.h"

namespace ghost_sensing {

class TCA9536 {
public:
    
    TCA9536(uint8_t I2c_address);
    void write_register(uint8_t reg, uint8_t value);
    uint8_t read_register(uint8_t reg);
private:
    uint8_t I2c_address;
};
}