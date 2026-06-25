#include <cstdint>
#include <memory>
#include "I2C_interfacing.h"
#include "TCA9536.h"

namespace ghost_sensing {


    TCA9536::TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint16_t len)
    : m_i2c_communication(iface), address(address), leng(len) {m_i2c_communication->init();}

    TCA9536::TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint8_t out, uint8_t config, uint16_t len)
    : m_i2c_communication(iface), address(address), leng(len) {m_i2c_communication->init();
        writeRegister(configuration, config);
        writeRegister(output, out);
    }

    TCA9536::TCA9536(std::shared_ptr<I2C_interfacing> iface, uint8_t address, uint8_t out, uint8_t pol, uint8_t config, uint8_t special, uint16_t len)
    : m_i2c_communication(iface), address(address), leng(len) {m_i2c_communication->init();
        writeRegister(configuration, config);
        writeRegister(output, out);
        writeRegister(polarity, pol);
        writeRegister(special_function, special);
    }

    TCA9536::~TCA9536()
    {
        m_i2c_communication->deinit();
    }
bool TCA9536::writeRegister(uint8_t reg, uint8_t data) {
    return (m_i2c_communication->write(reg, &data, 1) == 0);
}

bool TCA9536::readRegister(uint8_t reg, uint8_t &data) {
    return (m_i2c_communication->read(reg, &data, 1) == 0);
}

bool TCA9536::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    return (m_i2c_communication->read(reg, buf, len) == 0);
}



}; // namespace ghost_sensing
