#pragma once

#include <cstddef>
#include <cstdint>
#include "hardware/i2c.h"

// Abstract I2C bus interface.
//
// Return convention (mirrors the Pico SDK):
//   >= 0  number of bytes transferred
//   < 0   error (PICO_ERROR_GENERIC = -1, PICO_ERROR_TIMEOUT = -2)
class I2CBus {
public:
    virtual ~I2CBus() = default;

    virtual int write(uint8_t addr, const uint8_t *data, size_t len, bool nostop = false) = 0;
    virtual int read(uint8_t addr, uint8_t *dst,         size_t len, bool nostop = false) = 0;

    // Timeout variants. Default falls back to the blocking versions (timeout ignored).
    // HardwareI2CBus overrides these with i2c_write/read_timeout_us.
    virtual int write_timeout_us(uint8_t addr, const uint8_t *data, size_t len, bool nostop, uint32_t timeout_us) {
        return write(addr, data, len, nostop);
    }
    virtual int read_timeout_us(uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint32_t timeout_us) {
        return read(addr, dst, len, nostop);
    }

    virtual uint8_t     get_id()            const { return 255; }

    // Returns the underlying hardware I2C peripheral, or nullptr for PIO-based buses.
    virtual i2c_inst_t *get_i2c_instance() const { return nullptr; }

    bool is_hardware_bus() const { return is_hardware; }

protected:
    explicit I2CBus(bool is_hardware) : is_hardware(is_hardware) {}

private:
    bool is_hardware;
};
