/**
 ******************************************************************************
 * @file    platform.cpp
 * @brief   Pico SDK implementation of the VL53L4CD platform I2C layer.
 ******************************************************************************
 */

#include "platform.h"

#ifndef VL53L4CD_I2C_TIMEOUT_US
#define VL53L4CD_I2C_TIMEOUT_US  (50000)
#endif

uint8_t VL53L4CD::I2CWrite(uint16_t registerAddress, uint8_t *p_values, uint32_t size)
{
    uint8_t stack_buf[66];
    uint8_t *buf = stack_buf;
    bool allocated = false;

    if (size + 2 > sizeof(stack_buf)) {
        buf = (uint8_t *)malloc(size + 2);
        if (!buf) return 1;
        allocated = true;
    }

    buf[0] = (uint8_t)(registerAddress >> 8);
    buf[1] = (uint8_t)(registerAddress & 0xFF);
    memcpy(&buf[2], p_values, size);

    int ret = i2c_->write_timeout_us(address_, buf, size + 2, false,
                                   VL53L4CD_I2C_TIMEOUT_US);
    if (allocated) free(buf);

    return (ret == (int)(size + 2)) ? 0 : 1;
}

uint8_t VL53L4CD::I2CRead(uint16_t registerAddress, uint8_t *p_values, uint32_t size)
{
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(registerAddress >> 8);
    addr_buf[1] = (uint8_t)(registerAddress & 0xFF);

    int ret = i2c_->write_timeout_us(address_, addr_buf, 2, true,
                                   VL53L4CD_I2C_TIMEOUT_US);
    if (ret != 2) return 1;

    ret = i2c_->read_timeout_us(address_, p_values, size, false,
                              VL53L4CD_I2C_TIMEOUT_US);
    return (ret == (int)size) ? 0 : 1;
}

uint8_t VL53L4CD::RdDWord(uint16_t reg, uint32_t *value)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    uint8_t status = I2CRead(reg, buf, 4);
    if (!status) {
        *value = ((uint32_t)buf[0] << 24)
               + ((uint32_t)buf[1] << 16)
               + ((uint32_t)buf[2] <<  8)
               +  (uint32_t)buf[3];
    }
    return status;
}

uint8_t VL53L4CD::RdWord(uint16_t reg, uint16_t *value)
{
    uint8_t buf[2] = {0, 0};
    uint8_t status = I2CRead(reg, buf, 2);
    if (!status)
        *value = ((uint16_t)buf[0] << 8) + (uint16_t)buf[1];
    return status;
}

uint8_t VL53L4CD::RdByte(uint16_t reg, uint8_t *value)
{
    return I2CRead(reg, value, 1);
}

uint8_t VL53L4CD::WrByte(uint16_t reg, uint8_t value)
{
    return I2CWrite(reg, &value, 1);
}

uint8_t VL53L4CD::WrWord(uint16_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    return I2CWrite(reg, buf, 2);
}

uint8_t VL53L4CD::WrDWord(uint16_t reg, uint32_t value)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)((value >> 24) & 0xFF);
    buf[1] = (uint8_t)((value >> 16) & 0xFF);
    buf[2] = (uint8_t)((value >>  8) & 0xFF);
    buf[3] = (uint8_t)((value >>  0) & 0xFF);
    return I2CWrite(reg, buf, 4);
}

uint8_t VL53L4CD::WaitMs(uint32_t TimeMs)
{
    sleep_ms(TimeMs);
    return 0;
}
