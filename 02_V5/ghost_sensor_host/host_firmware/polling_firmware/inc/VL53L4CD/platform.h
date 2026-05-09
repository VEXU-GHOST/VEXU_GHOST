/**
 ******************************************************************************
 * @file    platform.h
 * @brief   Platform-specific definitions for VL53L4CD on Raspberry Pi Pico SDK.
 *
 *          This replaces the ST bare-metal platform.h. It maps the driver's
 *          vl53l4cd_dev_t handle to a struct that carries both the I2C address and a
 *          pointer to the Pico SDK i2c_inst_t, so multiple sensors on
 *          different buses are supported without globals.
 *
 * Copyright (c) 2023 STMicroelectronics – original API contract.
 * Pico SDK adaptation: public domain / your project license.
 ******************************************************************************
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

/* --------------------------------------------------------------------------
 * vl53l4cd_dev_t  –  device handle passed through every ST driver call
 *
 * The original ST ULD uses a plain uint16_t that holds the 8-bit I2C address
 * (left-shifted by 1, i.e. 0x52 for the default 0x29 7-bit address).
 *
 * For Pico SDK we need to also know *which* I2C peripheral to talk to,
 * so vl53l4cd_dev_t is a pointer to the struct below.  All the existing ST source
 * files (VL53L4CD_api.c, VL53L4CD_calibration.c) pass vl53l4cd_dev_t around opaquely
 * and never dereference it themselves – only the platform functions do.
 * -------------------------------------------------------------------------- */

typedef struct vl53l4cd_dev {
    i2c_inst_t *i2c;          /**< Pico I2C peripheral (i2c0 or i2c1)       */
    uint16_t    address;      /**< 7-bit I2C address (default 0x29)          */
} vl53l4cd_dev_s;

typedef vl53l4cd_dev_s *vl53l4cd_dev_t;

/** Compatibility alias – the unmodified ST driver sources use Dev_t. */
typedef vl53l4cd_dev_t  Dev_t;

/**
 * @brief Error type used by the driver.
 */
typedef uint8_t VL53L4CD_Error;

/* ---- Optional: uncomment to enable I2C Fast Mode Plus (up to 1 MHz) ---- */
/* #define VL53L4CD_I2C_FAST_MODE_PLUS */

/* ---- Platform I/O primitives expected by the ST driver ---- */

uint8_t VL53L4CD_RdDWord(vl53l4cd_dev_t dev, uint16_t registerAddr, uint32_t *value);
uint8_t VL53L4CD_RdWord (vl53l4cd_dev_t dev, uint16_t registerAddr, uint16_t *value);
uint8_t VL53L4CD_RdByte (vl53l4cd_dev_t dev, uint16_t registerAddr, uint8_t  *value);

uint8_t VL53L4CD_WrByte (vl53l4cd_dev_t dev, uint16_t registerAddr, uint8_t  value);
uint8_t VL53L4CD_WrWord (vl53l4cd_dev_t dev, uint16_t registerAddr, uint16_t value);
uint8_t VL53L4CD_WrDWord(vl53l4cd_dev_t dev, uint16_t registerAddr, uint32_t value);

uint8_t VL53L4CD_WaitMs (vl53l4cd_dev_t dev, uint32_t TimeMs);

/* ---- Low-level I2C helpers (used internally by the Rd/Wr functions) ---- */

uint8_t VL53L4CD_I2CWrite(vl53l4cd_dev_t dev, uint16_t registerAddress,
                           uint8_t *p_values, uint32_t size);
uint8_t VL53L4CD_I2CRead (vl53l4cd_dev_t dev, uint16_t registerAddress,
                           uint8_t *p_values, uint32_t size);

#endif /* _PLATFORM_H_ */