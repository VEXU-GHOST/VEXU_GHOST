/**
 ******************************************************************************
 * @file    icm20602_pico.h
 * @brief   Pico SDK I2C HAL callbacks for the ICM20602 driver.
 *
 *          Provides ready-made hal_wr / hal_rd / hal_sleep callbacks that
 *          use the device struct's `id` field to look up which I2C bus and
 *          address to use.  This lets you run multiple ICM20602 sensors on
 *          different buses or at different addresses without writing separate
 *          callback sets.
 *
 *          Usage:
 *            1) Register each sensor with icm20602_pico_register().
 *            2) Assign the returned id and the three callbacks to your
 *               icm20602_dev_t struct.
 *            3) Call icm20602_init() as normal.
 *
 *          Example:
 *            icm20602_dev_t imu = ICM20602_DEFAULT_INIT();
 *
 *            imu.id        = icm20602_pico_register(i2c0, 0x68);
 *            imu.hal_wr    = icm20602_pico_write;
 *            imu.hal_rd    = icm20602_pico_read;
 *            imu.hal_sleep = icm20602_pico_sleep;
 *
 *            icm20602_init(&imu);
 ******************************************************************************
 */

#ifndef ICM20602_PICO_H
#define ICM20602_PICO_H

#include <stdint.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "../src/open_source_drivers/icm20602/inc/icm20602.h"

/** Maximum number of ICM20602 sensors that can be registered. */
#define ICM20602_PICO_MAX_DEVS  4

/**
 * @brief Register an ICM20602 device and get back an id for it.
 *
 * @param i2c      Pico I2C peripheral (i2c0 or i2c1).
 * @param address  7-bit I2C address (0x68 or 0x69).
 * @return uint8_t Assigned id (0 .. ICM20602_PICO_MAX_DEVS-1),
 *                 or 0xFF if the table is full.
 */
uint8_t icm20602_pico_register(i2c_inst_t *i2c, uint8_t address);

/** HAL write callback – assign to dev.hal_wr */
int8_t icm20602_pico_write(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len);

/** HAL read callback – assign to dev.hal_rd */
int8_t icm20602_pico_read(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len);

/** HAL sleep callback – assign to dev.hal_sleep */
void icm20602_pico_sleep(uint32_t ms);

#endif /* ICM20602_PICO_H */