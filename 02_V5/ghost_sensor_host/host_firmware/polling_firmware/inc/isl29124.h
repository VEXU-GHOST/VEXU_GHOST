#ifndef ISL29124_H
#define ISL29124_H
 
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdbool.h>
 
// I2C address
#define ISL29124_I2C_ADDR           0x44
 
// Device ID
#define ISL29124_DEV_ID             0x7D
 
// Register map
#define DEVICE_ID_REG               0x00
#define CONFIG1_REG                 0x01
#define CONFIG2_REG                 0x02
#define CONFIG3_REG                 0x03
#define LOW_THRESHOLD_LBYTE_REG     0x04
#define LOW_THRESHOLD_HBYTE_REG     0x05
#define HIGH_THRESHOLD_LBYTE_REG    0x06
#define HIGH_THRESHOLD_HBYTE_REG    0x07
#define STATUS_FLAGS_REG            0x08
#define GREEN_DATA_LBYTE_REG        0x09
#define GREEN_DATA_HBYTE_REG        0x0A
#define RED_DATA_LBYTE_REG          0x0B
#define RED_DATA_HBYTE_REG          0x0C
#define BLUE_DATA_LBYTE_REG         0x0D
#define BLUE_DATA_HBYTE_REG         0x0E
 
// CONFIG1 bits
#define RGB_OP_MODE_CLEAR           0xF8
#define RGB_OP_PWDN_MODE_SET        0x00
#define RGB_OP_GREEN_MODE_SET       0x01
#define RGB_OP_RED_MODE_SET         0x02
#define RGB_OP_BLUE_MODE_SET        0x03
#define RGB_OP_STANDBY_MODE_SET     0x04
#define RGB_OP_GRB_MODE_SET         0x05
#define RGB_OP_GR_MODE_SET          0x06
#define RGB_OP_GB_MODE_SET          0x07
#define RGB_DATA_SENSE_RANGE_POS    3
#define RGB_SENSE_RANGE_330_SET     0xF7
#define RGB_SENSE_RANGE_4000_SET    0x08
#define ADC_RESOLUTION_BITS_POS     4
 
// CONFIG3 / interrupt bits
#define INTR_THRESHOLD_ASSIGN_POS   0
#define INTR_PERSIST_CTRL_POS       2
#define INTR_THRESHOLD_ASSIGN_CLEAR 0xFC
#define INTR_THRESHOLD_ASSIGN_GREEN 0x01
#define INTR_THRESHOLD_ASSIGN_RED   0x02
#define INTR_THRESHOLD_ASSIGN_BLUE  0x03
 
// STATUS FLAGS
#define BOUTF_FLAG_POS              2
#define RGBTHF_FLAG_POS             0x01
 
// CCT / CCM constants
#define NEW_CCM
 
#ifdef NEW_CCM
typedef enum { RangeLo = 0, RangeHi, RangeMax } range_t;
typedef enum { Bit16 = 0, Bit12, BitMax } resolution_t;
#endif
 
// Sensor data structure
typedef struct {
    i2c_inst_t *i2c;
    uint8_t  addr;
    uint8_t  als_range_using;   // 0 = 375 lux, 1 = 10000 lux
    uint8_t  adc_resolution;    // 0 = 16-bit, 1 = 12-bit
    uint16_t last_r;
    uint16_t last_g;
    uint16_t last_b;
    uint16_t cct;
    uint16_t X;
    uint16_t Y;
    uint16_t Z;
} isl29124_t;
 
// Public API
bool    isl29124_init(isl29124_t *dev, i2c_inst_t *i2c, uint8_t addr);
int     isl29124_write_byte(isl29124_t *dev, uint8_t reg, uint8_t val);
int     isl29124_read_byte(isl29124_t *dev, uint8_t reg, uint8_t *val);
int     isl29124_read_word16(isl29124_t *dev, uint8_t reg, uint16_t *val);
int     isl29124_write_word16(isl29124_t *dev, uint8_t reg, uint16_t val);
 
int     isl29124_read_rgb(isl29124_t *dev, uint16_t *r, uint16_t *g, uint16_t *b);
int     isl29124_set_mode(isl29124_t *dev, uint8_t mode);
int     isl29124_set_range(isl29124_t *dev, int range_lux);
int     isl29124_get_range(isl29124_t *dev, int *range_lux);
int     isl29124_set_resolution(isl29124_t *dev, int bits);
int     isl29124_get_resolution(isl29124_t *dev, int *bits);
void    isl29124_autorange(isl29124_t *dev, uint16_t green);
 
uint32_t isl29124_cal_cct(isl29124_t *dev);
uint32_t isl29124_cal_lux(isl29124_t *dev, int *cct);
 
#endif // ISL29124_H