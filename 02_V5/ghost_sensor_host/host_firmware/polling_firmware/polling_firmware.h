#pragma once

#include "pico/stdlib.h"

#ifndef __POLLING_FIRMWARE_H__
#define __POLLING_FIRMWARE_H__

#define I2C_PORT_0 i2c0
#define I2C_PORT_1 i2c1
#define SDA_INPUT_1 2
#define SCL_INPUT_1 3
#define SDA_INPUT_2 4
#define SCL_INPUT_2 5
#define SDA_INPUT_3 6
#define SCL_INPUT_3 7
#define SDA_INPUT_4 8
#define SCL_INPUT_4 9
#define SDA_INPUT_5 0
#define SCL_INPUT_5 1
#define SDA_INPUT_6 28
#define SCL_INPUT_6 29
#define SDA_INPUT_7 26
#define SCL_INPUT_7 27
#define SDA_INPUT_8 18
#define SCL_INPUT_8 19
#define BAUD_RATE 115200
#define I2C_FREQ_HZ 400000

/*

Rotary DAC I2C address translation
-----------------------------------
Each sensor sits behind a rotary DAC that remaps its I2C address. Switch
position 0xF leaves the sensor at its hardware default address; every other
position XORs a fixed offset onto that default. The offset depends only on the
DAC position, not on the sensor type, so a single table covers all sensors:

    actual_address = default_address ^ dac_address_offset[position]

Positions whose result falls outside the valid I2C range (0x07-0x78) are
rejected by is_valid_sensor_i2c().

Some positions are "invalid" not because they are special-cased, but because
the XOR result lands in the I2C reserved range. A 7-bit address is only usable
in 0x08-0x77 (the spec reserves 0x00-0x07 and 0x78-0x7F). When a DAC position
XORs the default down below 0x07, the result is a reserved address no device
can use:

    Sensor             default  invalid positions               result
    Color (ISL29125)   0x44     9 -> 0x44^0x45, B -> 0x44^0x40   0x01, 0x04
    IO expander (TCA)  0x41     9 -> 0x41^0x45, B -> 0x41^0x40   0x04, 0x01
    Distance (VL53)    0x29     4 -> 0x29^0x2F, 6 -> 0x29^0x2A   0x06, 0x03
    IMU (ICM20602)     0x68     (none)

The 0x4x defaults (color, IO expander) get pushed into reserved space by
positions 9 and B; the lower 0x29 distance default by positions 4 and 6; the
IMU's 0x68 is high enough that no offset ever drops it below 0x07.

MAKE SURE IO EXPANDERS HAVE THE ADDRESS TRANSLATION DAC STARTING AT C
IN ORDER TO NOT HAVE CONFLICTING ADDRESSES WITH COLOR SENSORS OTHERWISE
MAKE A NON-CONSECUTIVE ADDRESS TRANSLATION PATTERN ON THE IO EXPANDER IF
USING MORE THAN 4 IO EXPANDERS AND COLOR SENSORS

*/

// Default (DAC position 0xF) I2C address of each sensor type.
#define COLOR_SENSOR_DEFAULT_ADDR    0x44  // ISL29125
#define IMU_DEFAULT_ADDR             0x68  // ICM20602
#define DISTANCE_SENSOR_DEFAULT_ADDR 0x29  // VL53L4CD
#define IO_EXPANDER_DEFAULT_ADDR     0x41  // TCA9536

// IO expanders share an address range with color sensors, so their rotary DACs
// start at position 0xC to avoid conflicts (see note above).
#define IO_EXPANDER_DAC_START        0x0C

// XOR offset applied by the rotary DAC for each switch position 0x0-0xF.
const uint8_t dac_address_offset[16] = {
    0x7F, 0x75, 0x7A, 0x70, 0x2F, 0x25, 0x2A, 0x20,
    0x4F, 0x45, 0x4A, 0x40, 0x0F, 0x05, 0x0A, 0x00,
};

// Translate a rotary DAC switch position into the sensor's actual I2C address.
inline uint8_t dac_i2c_address(uint8_t default_addr, uint8_t dac_pos) {
    return default_addr ^ dac_address_offset[dac_pos & 0x0F];
}

#endif

