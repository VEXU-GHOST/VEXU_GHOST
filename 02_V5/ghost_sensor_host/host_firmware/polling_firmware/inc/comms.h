#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// ---------------------------------------------------------------------------
// Wire frame format (little-endian):
//   [0x55][0xAA][cmd:1][len_lo:1][len_hi:1][payload:N][checksum:1]
//
// checksum = XOR of bytes [cmd, len_lo, len_hi, payload...]
// Max payload: COMMS_MAX_PAYLOAD bytes (enforced to bound SRAM use).
//
// Command direction:
//   CMD_CONFIGURE  Jetson → Pico  flat YAML config blob
//   CMD_POLL_NOW   Jetson → Pico  trigger one poll with current config
//   CMD_ACK        Pico → Jetson  1-byte status: 0=OK, 1=ERR
//   CMD_DATA       Pico → Jetson  packed poll results batch
// ---------------------------------------------------------------------------

#define COMMS_MAGIC_0       0x55u
#define COMMS_MAGIC_1       0xAAu
#define CMD_CONFIGURE       0x01u
#define CMD_POLL_NOW        0x02u
#define CMD_ACK             0x03u
#define CMD_DATA            0x04u
#define COMMS_MAX_PAYLOAD   4096u

// ---------------------------------------------------------------------------
// SensorConfig — in-memory representation of a configuration YAML.
// Populated by comms_parse_yaml_config() from a CMD_CONFIGURE payload.
// ---------------------------------------------------------------------------

#define COMMS_MAX_SENSORS   5   // MAX_UNIQUES_DEVICES

struct SensorConfig {
    uint32_t polling_interval_ms;
    uint32_t polling_times;
    uint8_t  color_sensor_cnt;
    uint8_t  color_sensor_bus_sel[COMMS_MAX_SENSORS];
    uint8_t  imu_cnt;
    uint8_t  imu_bus_sel[COMMS_MAX_SENSORS];
    uint8_t  distance_sensor_cnt;
    uint8_t  distance_sensor_bus_sel[COMMS_MAX_SENSORS];
    uint8_t  io_expander_cnt;
    uint8_t  io_expander_bus_sel[COMMS_MAX_SENSORS];
    uint8_t  io_expander_pin_mode[COMMS_MAX_SENSORS][4]; // 255 = unset
};

// ---------------------------------------------------------------------------
// Packed sensor frames — used in CMD_DATA payload
// ---------------------------------------------------------------------------

struct __attribute__((packed)) ColorFrame {
    uint16_t r, g, b;
    uint32_t lux, cct;
};

struct __attribute__((packed)) ImuFrame {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;
};

struct __attribute__((packed)) DistanceFrame {
    uint16_t distance_mm;
    uint8_t  range_status;
    uint16_t sigma_mm;
    uint32_t signal_rate_kcps;
    uint32_t ambient_rate_kcps;
};

struct __attribute__((packed)) GpioFrame {
    uint8_t gpio_state;
};

// CMD_DATA payload header immediately followed by:
//   ColorFrame    [poll_count][color_sensor_cnt]
//   ImuFrame      [poll_count][imu_cnt]
//   DistanceFrame [poll_count][distance_sensor_cnt]
//   GpioFrame     [poll_count][io_expander_cnt]
struct __attribute__((packed)) DataHeader {
    uint32_t poll_count;
    uint8_t  color_cnt;
    uint8_t  imu_cnt;
    uint8_t  distance_cnt;
    uint8_t  gpio_cnt;
};

// ---------------------------------------------------------------------------
// Packet parser — call comms_feed_byte() for each received byte.
// Returns true when a complete, valid packet is ready to read.
// ---------------------------------------------------------------------------

bool        comms_feed_byte(uint8_t b);
uint8_t     comms_get_cmd();
const uint8_t *comms_get_payload();
uint16_t    comms_get_payload_len();

// ---------------------------------------------------------------------------
// YAML config parser — called by comms_task on CMD_CONFIGURE.
// Parses flat "key: value\n" lines from buf into cfg.
// Returns false if payload is malformed or any value is out-of-range.
// ---------------------------------------------------------------------------

bool comms_parse_yaml_config(const uint8_t *buf, uint16_t len, SensorConfig &cfg);

// ---------------------------------------------------------------------------
// Send a framed packet over USB CDC (stdout).
// ---------------------------------------------------------------------------

void comms_send(uint8_t cmd, const uint8_t *payload, uint16_t len);

// Convenience: send CMD_ACK with a single status byte (0=OK, 1=ERR).
static inline void comms_send_ack(uint8_t status) {
    comms_send(CMD_ACK, &status, 1);
}
