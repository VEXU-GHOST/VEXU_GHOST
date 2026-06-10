#pragma once

#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------------------
// Sensor Host serial protocol (see ghost_sensor_host/PROTOCOL.md).
//
// Wire frame:  COBS( [cmd:1][len_lo:1][len_hi:1][payload:len][checksum:1] ) 0x00
//   checksum = (cmd + len_lo + len_hi + sum(payload)) & 0xFF
// All multi-byte fields little-endian.
// ---------------------------------------------------------------------------

#define CMD_ACK            0x01u   // host -> ROS : [ref_cmd:1][status:1]
#define CMD_I2C_WRITE      0x10u   // ROS -> host : [port:1][addr:1][data:N]
#define CMD_READ_REQUEST   0x11u   // ROS -> host : see PROTOCOL.md
#define CMD_READ_RESULT    0x12u   // host -> ROS : [id:2][seq:2][status:1][data_len:1][data:N]

#define COMMS_MAX_PAYLOAD  512u

// Status codes (ACK / READ_RESULT)
#define ST_OK              0u
#define ST_BAD_PARAMS      1u
#define ST_WRITE_FAIL      2u
#define ST_READ_FAIL       3u
#define ST_TABLE_FULL      4u

// Feed one received byte. Returns true when a complete, valid frame is ready.
bool            comms_feed_byte(uint8_t b);
uint8_t         comms_get_cmd();
const uint8_t * comms_get_payload();
uint16_t        comms_get_payload_len();

// Send a framed packet over USB CDC (stdout).
void comms_send(uint8_t cmd, const uint8_t *payload, uint16_t len);
