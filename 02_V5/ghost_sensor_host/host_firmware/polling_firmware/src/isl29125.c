
// /*******************************************************************************
//  * isl29124.c - ISL29124 RGB light sensor driver for Raspberry Pi Pico SDK
//  *
//  * Ported from the original Linux kernel driver (Intersil Corporation, GPLv2).
//  * Replaces linux/i2c.h smbus calls with Pico SDK hardware/i2c.h calls.
//  *
//  * Wiring (default):
//  *   SDA -> GPIO 4  (i2c0)
//  *   SCL -> GPIO 5  (i2c0)
//  *   VCC -> 3.3V
//  *   GND -> GND
//  *
//  * Usage example at bottom of file / see isl29124.h for API.
//  ******************************************************************************/
 
// #include "../inc/isl29124.h"
// #include <stdio.h>
// #include <string.h>
 
// // ─── CCM tables (preserved exactly from original driver) ─────────────────────
 
// #ifdef NEW_CCM
 
// // 14-bit fixed point gain table [range][resolution]
// static int32_t CCM_Gain[RangeMax][BitMax] = {
//     {35447L, 631511L},  // RangeLo: Bit16, Bit12
//     {46172L,  22650L},  // RangeHi: Bit16, Bit12
// };
 
// static int32_t CCM_RangeLo[3][3] = {
//     { -2980L,  16389L, -11820L },  // X col
//     { -4388L,  16383L, -10653L },  // Y col
//     { -8998L,  13667L,  -3900L },  // Z col
// };
 
// static int32_t CCM_RangeHi[3][3] = {
//     {  -715L,  14265L,  -9230L },  // X col
//     { -3267L,  16383L,  -9969L },  // Y col
//     { -7420L,   7032L,   7344L },  // Z col
// };
 
// #endif // NEW_CCM
 
// // ─── Low-level I2C helpers ────────────────────────────────────────────────────
 
// /*
//  * Write a single byte to a register.
//  * Returns 0 on success, -1 on error.
//  */
// int isl29124_write_byte(isl29124_t *dev, uint8_t reg, uint8_t val)
// {
//     uint8_t buf[2] = { reg, val };
//     int ret = i2c_write_blocking(dev->i2c, dev->addr, buf, 2, false);
//     if (ret != 2) {
//         printf("[ISL29124] write_byte failed: reg=0x%02X val=0x%02X\n", reg, val);
//         return -1;
//     }
//     return 0;
// }
 
// /*
//  * Read a single byte from a register.
//  * Returns 0 on success, -1 on error.
//  */
// int isl29124_read_byte(isl29124_t *dev, uint8_t reg, uint8_t *val)
// {
//     // Write register address (no stop), then read 1 byte
//     int ret = i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true);
//     if (ret != 1) {
//         printf("[ISL29124] read_byte write phase failed: reg=0x%02X\n", reg);
//         return -1;
//     }
//     ret = i2c_read_blocking(dev->i2c, dev->addr, val, 1, false);
//     if (ret != 1) {
//         printf("[ISL29124] read_byte read phase failed: reg=0x%02X\n", reg);
//         return -1;
//     }
//     return 0;
// }
 
// /*
//  * Read a 16-bit little-endian word from two consecutive registers.
//  * Equivalent to i2c_smbus_read_i2c_block_data(..., 2, dat) in the original.
//  * Returns 0 on success, -1 on error.
//  */
// int isl29124_read_word16(isl29124_t *dev, uint8_t reg, uint16_t *val)
// {
//     uint8_t dat[2];
//     int ret = i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true);
//     if (ret != 1) {
//         printf("[ISL29124] read_word16 write phase failed: reg=0x%02X\n", reg);
//         return -1;
//     }
//     ret = i2c_read_blocking(dev->i2c, dev->addr, dat, 2, false);
//     if (ret != 2) {
//         printf("[ISL29124] read_word16 read phase failed: reg=0x%02X\n", reg);
//         return -1;
//     }
//     // Sensor sends LSB first
//     *val = ((uint16_t)dat[1] << 8) | (uint16_t)dat[0];
//     return 0;
// }
 
// /*
//  * Write a 16-bit value as two consecutive byte writes (LSB then MSB).
//  * Equivalent to two i2c_smbus_write_byte_data calls in the original.
//  * Returns 0 on success, -1 on error.
//  */
// int isl29124_write_word16(isl29124_t *dev, uint8_t reg, uint16_t val)
// {
//     uint8_t reg_l = val & 0xFF;
//     uint8_t reg_h = (val >> 8) & 0xFF;
 
//     if (isl29124_write_byte(dev, reg,     reg_l) < 0) return -1;
//     if (isl29124_write_byte(dev, reg + 1, reg_h) < 0) return -1;
//     return 0;
// }
 
// // ─── Configuration helpers ────────────────────────────────────────────────────
 
// int isl29124_set_mode(isl29124_t *dev, uint8_t mode)
// {
//     uint8_t reg;
//     if (isl29124_read_byte(dev, CONFIG1_REG, &reg) < 0) return -1;
//     reg &= RGB_OP_MODE_CLEAR;
//     reg |= (mode & 0x07);
//     return isl29124_write_byte(dev, CONFIG1_REG, reg);
// }
 
// int isl29124_set_range(isl29124_t *dev, int range_lux)
// {
//     uint8_t reg;
//     if (isl29124_read_byte(dev, CONFIG1_REG, &reg) < 0) return -1;
 
//     if (range_lux == 4000)
//         reg |= RGB_SENSE_RANGE_4000_SET;
//     else if (range_lux == 330)
//         reg &= RGB_SENSE_RANGE_330_SET;
//     else {
//         printf("[ISL29124] set_range: invalid range %d (use 330 or 4000)\n", range_lux);
//         return -1;
//     }
 
//     if (isl29124_write_byte(dev, CONFIG1_REG, reg) < 0) return -1;
//     dev->als_range_using = (range_lux == 4000) ? 1 : 0;
//     return 0;
// }
 
// int isl29124_get_range(isl29124_t *dev, int *range_lux)
// {
//     uint8_t reg;
//     if (isl29124_read_byte(dev, CONFIG1_REG, &reg) < 0) return -1;
//     *range_lux = (reg & (1 << RGB_DATA_SENSE_RANGE_POS)) ? 4000 : 330;
//     return 0;
// }
 
// int isl29124_set_resolution(isl29124_t *dev, int bits)
// {
//     uint8_t reg;
//     if (isl29124_read_byte(dev, CONFIG1_REG, &reg) < 0) return -1;
 
//     if (bits == 12)
//         reg |=  (1 << ADC_RESOLUTION_BITS_POS);
//     else if (bits == 16)
//         reg &= ~(1 << ADC_RESOLUTION_BITS_POS);
//     else {
//         printf("[ISL29124] set_resolution: invalid bits %d (use 12 or 16)\n", bits);
//         return -1;
//     }
 
//     if (isl29124_write_byte(dev, CONFIG1_REG, reg) < 0) return -1;
//     dev->adc_resolution = (bits == 12) ? 1 : 0;
//     return 0;
// }
 
// int isl29124_get_resolution(isl29124_t *dev, int *bits)
// {
//     uint8_t reg;
//     if (isl29124_read_byte(dev, CONFIG1_REG, &reg) < 0) return -1;
//     *bits = (reg & (1 << ADC_RESOLUTION_BITS_POS)) ? 12 : 16;
//     return 0;
// }
 
// // ─── Autorange ────────────────────────────────────────────────────────────────
 
// /*
//  * Automatically switch optical range based on green channel saturation.
//  * Mirrors the original autorange() logic exactly.
//  */
// void isl29124_autorange(isl29124_t *dev, uint16_t green)
// {
//     int range, res;
 
//     if (isl29124_get_resolution(dev, &res) < 0)  return;
//     if (isl29124_get_range(dev, &range) < 0)      return;
 
//     if (res == 12) {
//         if (range == 330  && green > 0x0CCC) isl29124_set_range(dev, 4000);
//         if (range == 4000 && green < 0x00CC) isl29124_set_range(dev, 330);
//     } else { // 16-bit
//         if (range == 330  && green > 0xCCCC) isl29124_set_range(dev, 4000);
//         if (range == 4000 && green < 0x0CCC) isl29124_set_range(dev, 330);
//     }
// }
 
// // ─── RGB read ─────────────────────────────────────────────────────────────────
 
// /*
//  * Read all three colour channels and cache them in the device struct.
//  * Returns 0 on success, -1 on error.
//  */
// int isl29124_read_rgb(isl29124_t *dev, uint16_t *r, uint16_t *g, uint16_t *b)
// {
//     if (isl29124_read_word16(dev, RED_DATA_LBYTE_REG,   r) < 0) return -1;
//     if (isl29124_read_word16(dev, GREEN_DATA_LBYTE_REG, g) < 0) return -1;
//     if (isl29124_read_word16(dev, BLUE_DATA_LBYTE_REG,  b) < 0) return -1;
 
//     dev->last_r = *r;
//     dev->last_g = *g;
//     dev->last_b = *b;
//     return 0;
// }
 
// // ─── CCT & Lux calculation ────────────────────────────────────────────────────
 
// /*
//  * Helper: signed 64-bit integer division (replaces kernel div64_s64).
//  */
// static inline int64_t div64(int64_t a, int64_t b)
// {
//     return a / b;
// }
 
// #ifdef NEW_CCM
 
// /*
//  * Calculate correlated colour temperature from the cached R/G/B values.
//  * Preserves the original NEW_CCM algorithm exactly.
//  * Returns CCT in Kelvin, or 0 if inputs are degenerate.
//  */
// uint32_t isl29124_cal_cct(isl29124_t *dev)
// {
//     int32_t cct;
//     int64_t X0, Y0, Z0, sum0;
//     int64_t x, y, n, xe, ye;
//     int64_t tmp;
//     uint8_t  range = dev->als_range_using;
//     uint8_t  bits  = 0;   // always 16-bit path per original (bits=0)
//     uint16_t als_r = dev->last_r;
//     uint16_t als_g = dev->last_g;
//     uint16_t als_b = dev->last_b;
 
//     if (range == 0) {
//         X0 = (int64_t)CCM_RangeLo[0][0]*als_r + (int64_t)CCM_RangeLo[0][1]*als_g + (int64_t)CCM_RangeLo[0][2]*als_b;
//         Y0 = (int64_t)CCM_RangeLo[1][0]*als_r + (int64_t)CCM_RangeLo[1][1]*als_g + (int64_t)CCM_RangeLo[1][2]*als_b;
//         Z0 = (int64_t)CCM_RangeLo[2][0]*als_r + (int64_t)CCM_RangeLo[2][1]*als_g + (int64_t)CCM_RangeLo[2][2]*als_b;
//     } else {
//         X0 = (int64_t)CCM_RangeHi[0][0]*als_r + (int64_t)CCM_RangeHi[0][1]*als_g + (int64_t)CCM_RangeHi[0][2]*als_b;
//         Y0 = (int64_t)CCM_RangeHi[1][0]*als_r + (int64_t)CCM_RangeHi[1][1]*als_g + (int64_t)CCM_RangeHi[1][2]*als_b;
//         Z0 = (int64_t)CCM_RangeHi[2][0]*als_r + (int64_t)CCM_RangeHi[2][1]*als_g + (int64_t)CCM_RangeHi[2][2]*als_b;
//     }
 
//     sum0 = X0 + Y0 + Z0;
//     if (sum0 == 0) {
//         printf("[ISL29124] cal_cct: sum0 is 0\n");
//         return 0;
//     }
 
//     x  = div64(X0 * 10000, sum0);
//     y  = div64(Y0 * 10000, sum0);
//     xe = 3320;  // 0.3320
//     ye = 1858;  // 0.1858
 
//     if (y == ye) {
//         printf("[ISL29124] cal_cct: y-ye is 0\n");
//         return 0;
//     }
 
//     n = div64((x - xe) * 10000, (y - ye));
 
//     // Robertson formula: CCT = -449n^3 + 3525n^2 - 6823n + 5520
//     tmp = div64(-449 * n, 10000);
//     tmp = div64((tmp + 3525) * n, 10000);
//     tmp = div64((tmp - 6823) * n, 10000);
//     cct = (int32_t)(tmp + 5520);
 
//     dev->X = (uint16_t)div64(X0, CCM_Gain[range][bits]);
//     dev->Y = (uint16_t)div64(Y0, CCM_Gain[range][bits]);
//     dev->Z = (uint16_t)div64(Z0, CCM_Gain[range][bits]);
 
//     if (cct < 0) cct = 0;
//     dev->cct = (uint16_t)cct;
//     return (uint32_t)cct;
// }
 
// /*
//  * Calculate lux and CCT together.
//  * Fills *cct and returns lux value.
//  */
// uint32_t isl29124_cal_lux(isl29124_t *dev, int *cct)
// {
//     uint32_t lux;
//     uint8_t  bits  = 0;
//     uint16_t r = dev->last_r;
//     uint16_t g = dev->last_g;
//     uint16_t b = dev->last_b;
 
//     if (dev->als_range_using == 0) {
//         // 375 lux range
//         lux = (uint32_t)(10 * ((int64_t)CCM_RangeLo[1][0]*r
//                              + (int64_t)CCM_RangeLo[1][1]*g
//                              + (int64_t)CCM_RangeLo[1][2]*b)
//                          / CCM_Gain[RangeLo][bits]);
//     } else {
//         // 10000 lux range
//         lux = (uint32_t)(10 * ((int64_t)CCM_RangeHi[1][0]*r
//                              + (int64_t)CCM_RangeHi[1][1]*g
//                              + (int64_t)CCM_RangeHi[1][2]*b)
//                          / CCM_Gain[RangeHi][bits]);
//     }
 
//     *cct = (int)isl29124_cal_cct(dev);
 
//     // Overflow guard from original driver
//     if (lux > 80000)
//         lux = 3u * dev->last_r;
 
//     if (r == 65535 && g == 65535 && b == 65535)
//         lux = 300000;
 
//     return lux;
// }
 
// #endif // NEW_CCM
 
// // ─── Initialisation ───────────────────────────────────────────────────────────
 
// /*
//  * Initialise the sensor and verify device ID.
//  * Call once after setting up the i2c bus.
//  *
//  * Example:
//  *   i2c_init(i2c0, 100000);
//  *   gpio_set_function(4, GPIO_FUNC_I2C);
//  *   gpio_set_function(5, GPIO_FUNC_I2C);
//  *   gpio_pull_up(4);
//  *   gpio_pull_up(5);
//  *
//  *   isl29124_t sensor;
//  *   isl29124_init(&sensor, i2c0, ISL29124_I2C_ADDR);
//  *
//  * Returns true on success, false if device not found or write failed.
//  */
// bool isl29124_init(isl29124_t *dev, i2c_inst_t *i2c, uint8_t addr)
// {
//     memset(dev, 0, sizeof(*dev));
//     dev->i2c  = i2c;
//     dev->addr = addr;
 
//     // Verify device ID
//     uint8_t id = 0;
//     sleep_ms(10);
//     if (isl29124_read_byte(dev, DEVICE_ID_REG, &id) < 0) {
//         printf("[ISL29124] init: failed to read device ID\n");
//         return false;
//     }
//     if (id != ISL29124_DEV_ID) {
//         printf("[ISL29124] init: unexpected device ID 0x%02X (expected 0x%02X)\n",
//                id, ISL29124_DEV_ID);
//         return false;
//     }
//     printf("[ISL29124] init: device ID OK (0x%02X)\n", id);
 
//     // CONFIG1: RGB mode, 375 lux range, 16-bit ADC
//     //   0x05 = GRB mode, 375 lux, 16-bit  (original polled init: 0x08 = GRB+375lux)
//     //   Use 0x05 here to match RGB_OP_GRB_MODE_SET with 375 lux default.
//     if (isl29124_write_byte(dev, CONFIG1_REG, 0x05) < 0) return false;
 
//     // CONFIG2: IR compensation disabled
//     if (isl29124_write_byte(dev, CONFIG2_REG, 0x00) < 0) return false;
 
//     // CONFIG3: no interrupt
//     if (isl29124_write_byte(dev, CONFIG3_REG, 0x00) < 0) return false;
 
//     // Clear brownout status flag
//     uint8_t status = 0;
//     if (isl29124_read_byte(dev, STATUS_FLAGS_REG, &status) < 0) return false;
//     status &= ~(1 << BOUTF_FLAG_POS);
//     if (isl29124_write_byte(dev, STATUS_FLAGS_REG, status) < 0) return false;
 
//     dev->als_range_using = 0;  // 375 lux
//     dev->adc_resolution  = 0;  // 16-bit
 
//     return true;
// }