/**
 ******************************************************************************
 * @file    icm20602_pico.c
 * @brief   Pico SDK I2C HAL callbacks for the ICM20602 driver.
 *
 *          Maintains a small lookup table that maps an id (0..N) to an
 *          {i2c_inst_t*, address} pair.  The hal_wr / hal_rd callbacks
 *          use the id passed in by the driver to find the right bus and
 *          address automatically.
 ******************************************************************************
 */

#include "../inc/icm20602_pico.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---- Device table ------------------------------------------------------- */

typedef struct {
    i2c_inst_t *i2c;
    uint8_t     address;
    bool        used;
} icm20602_pico_entry_t;

static icm20602_pico_entry_t dev_table[ICM20602_PICO_MAX_DEVS];
static uint8_t dev_count = 0;

/* ---- Registration ------------------------------------------------------- */

uint8_t icm20602_pico_register(i2c_inst_t *i2c, uint8_t address)
{
    if (dev_count >= ICM20602_PICO_MAX_DEVS) {
        return 0xFF;
    }

    uint8_t id = dev_count;
    dev_table[id].i2c     = i2c;
    dev_table[id].address = address;
    dev_table[id].used    = true;
    dev_count++;

    return id;
}

/* ---- HAL callbacks ------------------------------------------------------ */

int8_t icm20602_pico_write(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (id >= ICM20602_PICO_MAX_DEVS || !dev_table[id].used) {
        return -1;
    }

    i2c_inst_t *i2c = dev_table[id].i2c;
    uint8_t     addr = dev_table[id].address;

    /* Build [reg, data...] in a single buffer for one I2C transaction */
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    int ret = i2c_write_blocking(i2c, addr, buf, len + 1, false);
    return (ret == (int)(len + 1)) ? 0 : -1;
}

int8_t icm20602_pico_read(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (id >= ICM20602_PICO_MAX_DEVS || !dev_table[id].used) {
        return -1;
    }

    i2c_inst_t *i2c = dev_table[id].i2c;
    uint8_t     addr = dev_table[id].address;

    /* Write register address (repeated start), then read data */
    int ret = i2c_write_blocking(i2c, addr, &reg, 1, true);
    if (ret != 1) return -1;

    ret = i2c_read_blocking(i2c, addr, data, len, false);
    return (ret == (int)len) ? 0 : -1;
}

void icm20602_pico_sleep(uint32_t ms)
{
    sleep_ms(ms);
}