#include "HardwareI2CBus.h"

int HardwareI2CBus::write(uint8_t addr, const uint8_t *data, size_t len, bool nostop) {
    return i2c_write_blocking(i2c_, addr, data, len, nostop);
}

int HardwareI2CBus::read(uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
    return i2c_read_blocking(i2c_, addr, dst, len, nostop);
}

int HardwareI2CBus::write_timeout_us(uint8_t addr, const uint8_t *data, size_t len, bool nostop, uint32_t timeout_us) {
    return i2c_write_timeout_us(i2c_, addr, data, len, nostop, timeout_us);
}

int HardwareI2CBus::read_timeout_us(uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint32_t timeout_us) {
    return i2c_read_timeout_us(i2c_, addr, dst, len, nostop, timeout_us);
}
