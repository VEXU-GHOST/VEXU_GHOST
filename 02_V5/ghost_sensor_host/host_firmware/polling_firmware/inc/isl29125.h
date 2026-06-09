/*
Ported from the original Linux kernel driver (Intersil Corporation, GPLv2).
*/

#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "I2CBus.h"
#include <stdint.h>

// Device ID
#define ISL29125_DEV_ID             0x7D

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

class ISL29125 {
public:
    ISL29125() = default;

    bool init(I2CBus *i2c, uint8_t addr);

    int write_byte(uint8_t reg, uint8_t val);
    int read_byte(uint8_t reg, uint8_t &val);
    int read_word16(uint8_t reg, uint16_t &val);
    int write_word16(uint8_t reg, uint16_t val);

    int read_rgb(uint16_t &r, uint16_t &g, uint16_t &b);
    int set_mode(uint8_t mode);
    int set_range(int range_lux);
    int get_range(int &range_lux);
    int set_resolution(int bits);
    int get_resolution(int &bits);
    void autorange(uint16_t green);

    uint32_t cal_cct();
    uint32_t cal_lux(int &cct);

private:
    I2CBus *i2c_         = nullptr;
    uint8_t  addr_            = 0;
    uint8_t  als_range_using_ = 0;
    uint8_t  adc_resolution_  = 0;
    uint16_t last_r_          = 0;
    uint16_t last_g_          = 0;
    uint16_t last_b_          = 0;
    uint16_t cct_             = 0;
    uint16_t X_               = 0;
    uint16_t Y_               = 0;
    uint16_t Z_               = 0;
};
