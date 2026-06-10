#pragma once

#include "I2CBus.h"
#include "hardware/i2c.h"

// Wraps a Pico SDK hardware I2C peripheral (i2c0 or i2c1).
// Prerequisite: caller must call i2c_init() and configure the SDA/SCL GPIO
// pins before constructing this object.
class HardwareI2CBus final : public I2CBus {
public:
    explicit HardwareI2CBus(i2c_inst_t *i2c) : I2CBus(true), i2c_(i2c) {}

    int write(uint8_t addr, const uint8_t *data, size_t len, bool nostop = false) override;
    int read(uint8_t addr, uint8_t *dst,         size_t len, bool nostop = false) override;

    int write_timeout_us(uint8_t addr, const uint8_t *data, size_t len, bool nostop, uint32_t timeout_us) override;
    int read_timeout_us(uint8_t addr, uint8_t *dst,         size_t len, bool nostop, uint32_t timeout_us) override;

    i2c_inst_t *get_i2c_instance() const override { return i2c_; }

private:
    i2c_inst_t *i2c_;
};
