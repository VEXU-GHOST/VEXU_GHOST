// /**
//  ******************************************************************************
//  * @file    platform.c
//  * @brief   Pico SDK implementation of the VL53L4CD platform I/O layer.
//  *
//  *          Implements every function declared in platform.h using the Pico SDK
//  *          hardware_i2c API.  The VL53L4CD uses 16-bit register addresses sent
//  *          MSB-first, followed by one or more data bytes in the same
//  *          transaction.
//  *
//  * Copyright (c) 2023 STMicroelectronics – original API contract.
//  * Pico SDK adaptation: public domain / your project license.
//  ******************************************************************************
//  */

// #include "../../inc/VL53L4CD/platform.h"

// #include <stdlib.h>

// /* Default timeout for every I2C transaction (in microseconds).
//  * 50 ms is generous; individual byte transfers at 400 kHz take ~25 µs.
//  * Increase this if you run at a very low clock rate.                        */
// #ifndef VL53L4CD_I2C_TIMEOUT_US
// #define VL53L4CD_I2C_TIMEOUT_US  (50000)
// #endif

// /* ========================================================================= */
// /*  Low-level I2C helpers                                                    */
// /* ========================================================================= */

// /**
//  * @brief Write `size` bytes starting at `registerAddress`.
//  *
//  * The VL53L4CD expects a 2-byte (big-endian) register address followed by the
//  * data payload, all within one I2C START…STOP frame.  We build that in a
//  * small stack buffer for short writes (≤ 64 data bytes, covering every
//  * register write the driver makes) and fall back to a heap allocation for
//  * the rare bulk-write during SensorInit() which sends ~91 bytes.
//  */
// uint8_t VL53L4CD_I2CWrite(vl53l4cd_dev_t dev, uint16_t registerAddress,
//                            uint8_t *p_values, uint32_t size)
// {
//     /* 2 address bytes + payload ------------------------------------------- */
//     uint8_t stack_buf[66];            /* enough for most writes              */
//     uint8_t *buf = stack_buf;
//     int     allocated = 0;

//     if (size + 2 > sizeof(stack_buf)) {
//         buf = (uint8_t *)malloc(size + 2);
//         if (!buf) return 1;          /* allocation failed                    */
//         allocated = 1;
//     }

//     buf[0] = (uint8_t)(registerAddress >> 8);
//     buf[1] = (uint8_t)(registerAddress & 0xFF);
//     memcpy(&buf[2], p_values, size);

//     int ret = i2c_write_timeout_us(dev->i2c, dev->address,
//                                    buf, size + 2,
//                                    false,               /* send STOP        */
//                                    VL53L4CD_I2C_TIMEOUT_US);

//     if (allocated) free(buf);

//     /* Pico SDK returns the number of bytes written, or PICO_ERROR_GENERIC /
//      * PICO_ERROR_TIMEOUT on failure.                                        */
//     return (ret == (int)(size + 2)) ? 0 : 1;
// }

// /**
//  * @brief Read `size` bytes starting at `registerAddress`.
//  *
//  * Two-phase I2C transaction:
//  *   1) Write the 2-byte register address (no STOP – repeated start).
//  *   2) Read `size` bytes into `p_values` (with STOP).
//  */
// uint8_t VL53L4CD_I2CRead(vl53l4cd_dev_t dev, uint16_t registerAddress,
//                           uint8_t *p_values, uint32_t size)
// {
//     int ret;
//     uint8_t addr_buf[2];

//     addr_buf[0] = (uint8_t)(registerAddress >> 8);
//     addr_buf[1] = (uint8_t)(registerAddress & 0xFF);

//     /* Phase 1 – write register address, keep bus held (no STOP) */
//     ret = i2c_write_timeout_us(dev->i2c, dev->address,
//                                addr_buf, 2,
//                                true,                    /* no STOP          */
//                                VL53L4CD_I2C_TIMEOUT_US);
//     if (ret != 2) return 1;

//     /* Phase 2 – read data */
//     ret = i2c_read_timeout_us(dev->i2c, dev->address,
//                               p_values, size,
//                               false,                    /* send STOP        */
//                               VL53L4CD_I2C_TIMEOUT_US);
//     return (ret == (int)size) ? 0 : 1;
// }

// /* ========================================================================= */
// /*  Register-width helpers (called by the ST driver source)                  */
// /* ========================================================================= */

// uint8_t VL53L4CD_RdDWord(vl53l4cd_dev_t dev, uint16_t RegisterAdress, uint32_t *value)
// {
//     uint8_t status;
//     uint8_t buffer[4] = {0, 0, 0, 0};

//     status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 4);
//     if (!status) {
//         *value = ((uint32_t)buffer[0] << 24)
//                + ((uint32_t)buffer[1] << 16)
//                + ((uint32_t)buffer[2] <<  8)
//                +  (uint32_t)buffer[3];
//     }
//     return status;
// }

// uint8_t VL53L4CD_RdWord(vl53l4cd_dev_t dev, uint16_t RegisterAdress, uint16_t *value)
// {
//     uint8_t status;
//     uint8_t buffer[2] = {0, 0};
//     status = VL53L4CD_I2CRead(dev, RegisterAdress, buffer, 2);
//     if (!status) {
//         *value = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];
//     }
//     return status;
// }

// uint8_t VL53L4CD_RdByte(vl53l4cd_dev_t dev, uint16_t RegisterAdress, uint8_t *value)
// {
//     return VL53L4CD_I2CRead(dev, RegisterAdress, value, 1);
// }

// uint8_t VL53L4CD_WrByte(vl53l4cd_dev_t dev, uint16_t RegisterAdress, uint8_t value)
// {
//     return VL53L4CD_I2CWrite(dev, RegisterAdress, &value, 1);
// }

// uint8_t VL53L4CD_WrWord(vl53l4cd_dev_t dev, uint16_t RegisterAdress, uint16_t value)
// {
//     uint8_t buffer[2];

//     buffer[0] = (uint8_t)(value >> 8);
//     buffer[1] = (uint8_t)(value & 0xFF);
//     return VL53L4CD_I2CWrite(dev, RegisterAdress, buffer, 2);
// }

// uint8_t VL53L4CD_WrDWord(vl53l4cd_dev_t dev, uint16_t RegisterAdress, uint32_t value)
// {
//     uint8_t buffer[4];

//     buffer[0] = (uint8_t)((value >> 24) & 0xFF);
//     buffer[1] = (uint8_t)((value >> 16) & 0xFF);
//     buffer[2] = (uint8_t)((value >>  8) & 0xFF);
//     buffer[3] = (uint8_t)((value >>  0) & 0xFF);
//     return VL53L4CD_I2CWrite(dev, RegisterAdress, buffer, 4);
// }

// /* ========================================================================= */
// /*  Delay                                                                    */
// /* ========================================================================= */

// uint8_t VL53L4CD_WaitMs(vl53l4cd_dev_t dev, uint32_t TimeMs)
// {
//     (void)dev;
//     sleep_ms(TimeMs);
//     return 0;
// }