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

Color sensor address translations on the rotary DAC:

0 - 3b
1 - 31
2 - 3e
3 - 34
4 - 6b
5 - 61
6 - 6e
7 - 64
8 - 0b
9 - invalid
A - 0e
B - invalid
C - 4b
D - 41
E - 4e
F - 44

IMU address translations on the rotary DAC:

0 - 17
1 - 1d
2 - 12
3 - 18
4 - 47
5 - 4d
6 - 42
7 - 48
8 - 27
9 - 2d
A - 22
B - 28
C - 67
D - 6d
E - 62
F - 68

Distance sensor address translations on the rotary DAC:

0 - 56
1 - 5c
2 - 53
3 - 59
4 - inavlid
5 - 0c
6 - invalid
7 - 09
8 - 66
9 - 6c
A - 63
B - 69
C - 26
D - 2c
E - 23
F - 29

IO Expander address translations on the rotary DAC:

0 - 3e
1 - 34
2 - 3b
3 - 31
4 - 6e
5 - 64
6 - 6b
7 - 61
8 - 0e
9 - invalid
A - 0b
B - invalid
C - 4e
D - 44
E - 4b
F - 41

0xFF means invalid address (valid address range is 0x07-0x78)

MAKE SURE IO EXPANDERS HAVE THE ADDRESS TRANSLATION DAC STARTING AT C
IN ORDER TO NOT HAVE CONFLICTING ADDRESSES WITH COLOR SENSORS OTHERWISE
MAKE A NON-CONSECUTIVE ADDRESS TRANSLATION PATTERN ON THE IO EXPANDER IF
USING MORE THAN 4 IO EXPANDERS AND COLOR SENSORS

*/

const uint8_t color_sensor_address[16] = {0x3B, 0x31, 0x3E, 0x34, 0x6B, 0x61, 0x6E, 0x64, 0x0B, 0xFF, 0x0E, 0xFF, 0x4B, 0x41, 0x4E, 0x44};
const uint8_t imu_address[16] = {0x17, 0x1D, 0x12, 0x18, 0x47, 0x4D, 0x42, 0x48, 0x27, 0x2D, 0x22, 0x28, 0x67, 0x6D, 0x62, 0x68};
const uint8_t distance_sensor_address[16] = {0x56, 0x5C, 0x53, 0x59, 0xFF, 0x0C, 0xFF, 0x09, 0x06, 0x6C, 0x63, 0x69, 0x26, 0x2C, 0x23, 0x29};
const uint8_t io_expander_address[16] = {0x4E, 0x44, 0x4B, 0x41, 0x3E, 0x34, 0x3B, 0x31, 0x6E, 0x64, 0x6B, 0x61, 0x0E, 0xFF, 0x0B, 0xFF};

#endif

static void reset_device_array();
