/*******************************************************************************
 * isl29125.cpp - ISL29125 RGB light sensor driver for Raspberry Pi Pico SDK
 *
 * Ported from the original Linux kernel driver (Intersil Corporation, GPLv2).
 * Replaces linux/i2c.h smbus calls with Pico SDK hardware/i2c.h calls.
 ******************************************************************************/

#include "isl29125.h"
#include <stdio.h>

// ─── CCM tables (preserved exactly from original driver) ─────────────────────

#ifdef NEW_CCM

// 14-bit fixed point gain table [range][resolution]
static int32_t CCM_Gain[RangeMax][BitMax] = {
    {35447L, 631511L},  // RangeLo: Bit16, Bit12
    {46172L,  22650L},  // RangeHi: Bit16, Bit12
};

static int32_t CCM_RangeLo[3][3] = {
    { -2980L,  16389L, -11820L },  // X col
    { -4388L,  16383L, -10653L },  // Y col
    { -8998L,  13667L,  -3900L },  // Z col
};

static int32_t CCM_RangeHi[3][3] = {
    {  -715L,  14265L,  -9230L },  // X col
    { -3267L,  16383L,  -9969L },  // Y col
    { -7420L,   7032L,   7344L },  // Z col
};

#endif // NEW_CCM

// ─── Low-level I2C helpers ────────────────────────────────────────────────────

int ISL29125::write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (i2c_ == nullptr) {
        printf("[ISL29125] write_byte failed: uninitialized I2C bus\n");
        return -1;
    }
    int ret = i2c_->write(addr_, buf, 2, false);
    if (ret != 2) {
        printf("[ISL29125] write_byte failed: reg=0x%02X val=0x%02X\n", reg, val);
        return -1;
    }
    return 0;
}

int ISL29125::read_byte(uint8_t reg, uint8_t &val)
{
    if (i2c_ == nullptr) {
        printf("[ISL29125] write_byte failed: uninitialized I2C bus\n");
        return -1;
    }
    int ret = i2c_->write(addr_, &reg, 1, true);
    if (ret != 1) {
        printf("[ISL29125] read_byte write phase failed: reg=0x%02X\n", reg);
        return -1;
    }
    ret = i2c_->read(addr_, &val, 1, false);
    if (ret != 1) {
        printf("[ISL29125] read_byte read phase failed: reg=0x%02X\n", reg);
        return -1;
    }
    return 0;
}

int ISL29125::read_word16(uint8_t reg, uint16_t &val)
{
    if (i2c_ == nullptr) {
        printf("[ISL29125] write_byte failed: uninitialized I2C bus\n");
        return -1;
    }
    uint8_t dat[2];
    int ret = i2c_->write(addr_, &reg, 1, true);
    if (ret != 1) {
        printf("[ISL29125] read_word16 write phase failed: reg=0x%02X\n", reg);
        return -1;
    }
    ret = i2c_->read(addr_, dat, 2, false);
    if (ret != 2) {
        printf("[ISL29125] read_word16 read phase failed: reg=0x%02X\n", reg);
        return -1;
    }
    val = ((uint16_t)dat[1] << 8) | (uint16_t)dat[0];
    return 0;
}

int ISL29125::write_word16(uint8_t reg, uint16_t val)
{
    if (write_byte(reg,     val & 0xFF)        < 0) return -1;
    if (write_byte(reg + 1, (val >> 8) & 0xFF) < 0) return -1;
    return 0;
}

// ─── Configuration helpers ────────────────────────────────────────────────────

int ISL29125::set_mode(uint8_t mode)
{
    uint8_t reg;
    if (read_byte(CONFIG1_REG, reg) < 0) return -1;
    reg &= RGB_OP_MODE_CLEAR;
    reg |= (mode & 0x07);
    return write_byte(CONFIG1_REG, reg);
}

int ISL29125::set_range(int range_lux)
{
    uint8_t reg;
    if (read_byte(CONFIG1_REG, reg) < 0) return -1;

    if (range_lux == 4000)
        reg |= RGB_SENSE_RANGE_4000_SET;
    else if (range_lux == 330)
        reg &= RGB_SENSE_RANGE_330_SET;
    else {
        printf("[ISL29125] set_range: invalid range %d (use 330 or 4000)\n", range_lux);
        return -1;
    }

    if (write_byte(CONFIG1_REG, reg) < 0) return -1;
    als_range_using_ = (range_lux == 4000) ? 1 : 0;
    return 0;
}

int ISL29125::get_range(int &range_lux)
{
    uint8_t reg;
    if (read_byte(CONFIG1_REG, reg) < 0) return -1;
    range_lux = (reg & (1 << RGB_DATA_SENSE_RANGE_POS)) ? 4000 : 330;
    return 0;
}

int ISL29125::set_resolution(int bits)
{
    uint8_t reg;
    if (read_byte(CONFIG1_REG, reg) < 0) return -1;

    if (bits == 12)
        reg |=  (1 << ADC_RESOLUTION_BITS_POS);
    else if (bits == 16)
        reg &= ~(1 << ADC_RESOLUTION_BITS_POS);
    else {
        printf("[ISL29125] set_resolution: invalid bits %d (use 12 or 16)\n", bits);
        return -1;
    }

    if (write_byte(CONFIG1_REG, reg) < 0) return -1;
    adc_resolution_ = (bits == 12) ? 1 : 0;
    return 0;
}

int ISL29125::get_resolution(int &bits)
{
    uint8_t reg;
    if (read_byte(CONFIG1_REG, reg) < 0) return -1;
    bits = (reg & (1 << ADC_RESOLUTION_BITS_POS)) ? 12 : 16;
    return 0;
}

// ─── Autorange ────────────────────────────────────────────────────────────────

void ISL29125::autorange(uint16_t green)
{
    int range, res;

    if (get_resolution(res)  < 0) return;
    if (get_range(range)     < 0) return;

    if (res == 12) {
        if (range == 330  && green > 0x0CCC) set_range(4000);
        if (range == 4000 && green < 0x00CC) set_range(330);
    } else {
        if (range == 330  && green > 0xCCCC) set_range(4000);
        if (range == 4000 && green < 0x0CCC) set_range(330);
    }
}

// ─── RGB read ─────────────────────────────────────────────────────────────────

int ISL29125::read_rgb(uint16_t &r, uint16_t &g, uint16_t &b)
{
    if (read_word16(RED_DATA_LBYTE_REG,   r) < 0) return -1;
    if (read_word16(GREEN_DATA_LBYTE_REG, g) < 0) return -1;
    if (read_word16(BLUE_DATA_LBYTE_REG,  b) < 0) return -1;

    last_r_ = r;
    last_g_ = g;
    last_b_ = b;
    return 0;
}

// ─── CCT & Lux calculation ────────────────────────────────────────────────────

static inline int64_t div64(int64_t a, int64_t b)
{
    return a / b;
}

#ifdef NEW_CCM

uint32_t ISL29125::cal_cct()
{
    int32_t cct;
    int64_t X0, Y0, Z0, sum0;
    int64_t x, y, n, xe, ye;
    int64_t tmp;
    uint8_t  range = als_range_using_;
    uint8_t  bits  = 0;
    uint16_t als_r = last_r_;
    uint16_t als_g = last_g_;
    uint16_t als_b = last_b_;

    if (range == 0) {
        X0 = (int64_t)CCM_RangeLo[0][0]*als_r + (int64_t)CCM_RangeLo[0][1]*als_g + (int64_t)CCM_RangeLo[0][2]*als_b;
        Y0 = (int64_t)CCM_RangeLo[1][0]*als_r + (int64_t)CCM_RangeLo[1][1]*als_g + (int64_t)CCM_RangeLo[1][2]*als_b;
        Z0 = (int64_t)CCM_RangeLo[2][0]*als_r + (int64_t)CCM_RangeLo[2][1]*als_g + (int64_t)CCM_RangeLo[2][2]*als_b;
    } else {
        X0 = (int64_t)CCM_RangeHi[0][0]*als_r + (int64_t)CCM_RangeHi[0][1]*als_g + (int64_t)CCM_RangeHi[0][2]*als_b;
        Y0 = (int64_t)CCM_RangeHi[1][0]*als_r + (int64_t)CCM_RangeHi[1][1]*als_g + (int64_t)CCM_RangeHi[1][2]*als_b;
        Z0 = (int64_t)CCM_RangeHi[2][0]*als_r + (int64_t)CCM_RangeHi[2][1]*als_g + (int64_t)CCM_RangeHi[2][2]*als_b;
    }

    sum0 = X0 + Y0 + Z0;
    if (sum0 == 0) {
        // printf("[ISL29125] cal_cct: sum0 is 0\n");
        return 0;
    }

    x  = div64(X0 * 10000, sum0);
    y  = div64(Y0 * 10000, sum0);
    xe = 3320;
    ye = 1858;

    if (y == ye) {
        printf("[ISL29125] cal_cct: y-ye is 0\n");
        return 0;
    }

    n = div64((x - xe) * 10000, (y - ye));

    // Robertson formula: CCT = -449n^3 + 3525n^2 - 6823n + 5520
    tmp = div64(-449 * n, 10000);
    tmp = div64((tmp + 3525) * n, 10000);
    tmp = div64((tmp - 6823) * n, 10000);
    cct = (int32_t)(tmp + 5520);

    X_ = (uint16_t)div64(X0, CCM_Gain[range][bits]);
    Y_ = (uint16_t)div64(Y0, CCM_Gain[range][bits]);
    Z_ = (uint16_t)div64(Z0, CCM_Gain[range][bits]);

    if (cct < 0) cct = 0;
    cct_ = (uint16_t)cct;
    return (uint32_t)cct;
}

uint32_t ISL29125::cal_lux(int &cct)
{
    uint32_t lux;
    uint8_t  bits  = 0;
    uint16_t r = last_r_;
    uint16_t g = last_g_;
    uint16_t b = last_b_;

    if (als_range_using_ == 0) {
        lux = (uint32_t)(10 * ((int64_t)CCM_RangeLo[1][0]*r
                             + (int64_t)CCM_RangeLo[1][1]*g
                             + (int64_t)CCM_RangeLo[1][2]*b)
                         / CCM_Gain[RangeLo][bits]);
    } else {
        lux = (uint32_t)(10 * ((int64_t)CCM_RangeHi[1][0]*r
                             + (int64_t)CCM_RangeHi[1][1]*g
                             + (int64_t)CCM_RangeHi[1][2]*b)
                         / CCM_Gain[RangeHi][bits]);
    }

    cct = (int)cal_cct();

    if (lux > 80000)
        lux = 3u * last_r_;

    if (r == 65535 && g == 65535 && b == 65535)
        lux = 300000;

    return lux;
}

#endif // NEW_CCM

// ─── Initialisation ───────────────────────────────────────────────────────────

bool ISL29125::init(I2CBus *i2c, uint8_t addr)
{
    i2c_  = i2c;
    addr_ = addr;

    uint8_t id = 0;
    sleep_ms(10);
    if (read_byte(DEVICE_ID_REG, id) < 0) {
        printf("[ISL29125] init: failed to read device ID\n");
        return false;
    }
    if (id != ISL29125_DEV_ID) {
        printf("[ISL29125] init: unexpected device ID 0x%02X (expected 0x%02X)\n",
               id, ISL29125_DEV_ID);
        return false;
    }
    // printf("[ISL29125] init: device ID OK (0x%02X)\n", id);

    if (write_byte(CONFIG1_REG, 0x05) < 0) return false;
    if (write_byte(CONFIG2_REG, 0x00) < 0) return false;
    if (write_byte(CONFIG3_REG, 0x00) < 0) return false;

    uint8_t status = 0;
    if (read_byte(STATUS_FLAGS_REG, status) < 0) return false;
    status &= ~(1 << BOUTF_FLAG_POS);
    if (write_byte(STATUS_FLAGS_REG, status) < 0) return false;

    als_range_using_ = 0;
    adc_resolution_  = 0;

    return true;
}
