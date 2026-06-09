#include "comms.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static size_t cobsEncode(const void *data, size_t length, uint8_t *buffer);
static size_t cobsDecode(const uint8_t *buffer, size_t length, void *data);

// ---------------------------------------------------------------------------
// Packet parser
// ---------------------------------------------------------------------------

namespace {

static uint8_t  s_cobs_buf[COMMS_MAX_MSG_LEN + 2];
static uint16_t s_cobs_len     = 0;
static bool     s_packet_ready = false;
static uint8_t  s_cmd          = 0;
static uint16_t s_len          = 0;
static uint8_t  s_payload[COMMS_MAX_PAYLOAD];

} // namespace

bool comms_feed_byte(uint8_t b) {
    s_packet_ready = false;

    if (b == 0x00) {
        // COBS frame complete — decode then validate
        if (s_cobs_len > 0) {
            static uint8_t decoded[COMMS_MAX_MSG_LEN];
            size_t decoded_len = cobsDecode(s_cobs_buf, s_cobs_len, decoded);

            // Decoded layout: [magic x6][cmd:1][len_lo:1][len_hi:1][payload:N][checksum:1]
            const uint16_t HEADER_LEN = 9; // 6 magic + cmd + len_lo + len_hi
            if (decoded_len >= HEADER_LEN + 1 &&
                decoded[0] == COMMS_MAGIC_IN_0 &&
                decoded[1] == COMMS_MAGIC_IN_1 &&
                decoded[2] == COMMS_MAGIC_IN_2 &&
                decoded[3] == COMMS_MAGIC_IN_3 &&
                decoded[4] == COMMS_MAGIC_IN_4 &&
                decoded[5] == COMMS_MAGIC_IN_5)
            {
                uint16_t payload_len = decoded[7] | (static_cast<uint16_t>(decoded[8]) << 8);

                if (payload_len <= COMMS_MAX_PAYLOAD &&
                    decoded_len == HEADER_LEN + payload_len + 1)
                {
                    uint8_t cs = decoded[6] + decoded[7] + decoded[8];
                    for (uint16_t i = 0; i < payload_len; i++) cs += decoded[9 + i];

                    if (cs == decoded[HEADER_LEN + payload_len]) {
                        s_cmd = decoded[6];
                        s_len = payload_len;
                        memcpy(s_payload, decoded + HEADER_LEN, payload_len);
                        s_packet_ready = true;
                    }
                }
            }
        }
        s_cobs_len = 0;
    } else {
        if (s_cobs_len < sizeof(s_cobs_buf)) {
            s_cobs_buf[s_cobs_len++] = b;
        } else {
            s_cobs_len = 0;  // overflow, reset
        }
    }

    return s_packet_ready;
}

uint8_t comms_get_cmd()                { return s_cmd; }
const uint8_t *comms_get_payload()     { return s_payload; }
uint16_t comms_get_payload_len()       { return s_len; }

// ---------------------------------------------------------------------------
// Send a framed packet over USB CDC via stdout
// ---------------------------------------------------------------------------

void comms_send(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint32_t raw_message_length = len + 10; // 2 bytes for payload length, 1 byte for cmd, 1 byte for checksum, and 6 bytes for start sequence
    static uint8_t message[COMMS_MAX_OUT_MSG_LEN];
    uint8_t cs = cmd + (uint8_t)(len & 0xFF) + (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) cs += payload[i];
    message[0] = COMMS_MAGIC_OUT_0;
    message[1] = COMMS_MAGIC_OUT_1;
    message[2] = COMMS_MAGIC_OUT_2;
    message[3] = COMMS_MAGIC_OUT_3;
    message[4] = COMMS_MAGIC_OUT_4;
    message[5] = COMMS_MAGIC_OUT_5;
    message[6] = cmd;
    message[7] = (uint8_t)(len & 0xFF);
    message[8] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) {
        message[i + 9] = payload[i];
    }
    message[raw_message_length - 1] = cs;
    static uint8_t encoded_message[COMMS_MAX_OUT_MSG_LEN + 2];
    memset(encoded_message, 0, sizeof(encoded_message));
    uint32_t encoded_len = cobsEncode(message, raw_message_length, encoded_message);
    for (uint32_t i = 0; i < encoded_len; i++) {
        putchar_raw(encoded_message[i]);
    }
    putchar_raw(0x00);  // COBS frame delimiter — receivers frame on this null byte
    fflush(stdout);
}

// ---------------------------------------------------------------------------
// YAML config parser
// ---------------------------------------------------------------------------
//
// Supported format: one "key: value\n" per line (LF or CRLF).
// Array values are comma-separated integers, e.g. "2,3,4,5,2".
// Unknown keys are ignored. Leading/trailing whitespace on values is stripped.

namespace {

// Parse up to `n` comma-separated uint8_t values from `src` into `dst`.
static uint8_t parse_u8_array(char *src, uint8_t *dst, uint8_t n) {
    uint8_t count = 0;
    char *tok = src;
    char *end = src + strlen(src);
    while (tok < end && count < n) {
        char *comma = (char *)memchr(tok, ',', (size_t)(end - tok));
        if (comma) *comma = '\0';
        dst[count++] = (uint8_t)strtoul(tok, nullptr, 10);
        if (!comma) break;
        tok = comma + 1;
    }
    return count;
}

static uint8_t parse_u16_array(char *src, uint16_t *dst, uint8_t n) {
    uint8_t count = 0;
    char *tok = src;
    char *end = src + strlen(src);
    while (tok < end && count < n) {
        char *comma = (char *)memchr(tok, ',', (size_t)(end - tok));
        if (comma) *comma = '\0';
        dst[count++] = (uint16_t)strtoul(tok, nullptr, 10);
        if (!comma) break;
        tok = comma + 1;
    }
    return count;
}

// Strip leading and trailing whitespace in-place; returns pointer to first non-space.
static char *strip(char *s) {
    while (*s == ' ' || *s == '\t') s++;
    char *end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
        *--end = '\0';
    return s;
}

} // namespace

bool comms_parse_yaml_config(const uint8_t *buf, uint16_t len, SensorConfig &cfg) {
    // Work on a mutable copy (max COMMS_MAX_PAYLOAD, but practically much smaller).
    static char work[COMMS_MAX_PAYLOAD + 1];
    if (len > COMMS_MAX_PAYLOAD) return false;
    memcpy(work, buf, len);
    work[len] = '\0';

    char *line = work;
    char *file_end = work + len;

    while (line < file_end) {
        // Find end of line
        char *nl = (char *)memchr(line, '\n', (size_t)(file_end - line));
        if (nl) *nl = '\0';
        else    nl = file_end - 1;  // last line with no newline

        // Split on first ':'
        char *colon = strchr(line, ':');
        if (colon) {
            *colon = '\0';
            char *key = strip(line);
            char *val = strip(colon + 1);

            if      (strcmp(key, "polling_interval_ms")              == 0) cfg.polling_interval_ms                 = (uint32_t)strtoul(val, nullptr, 10);
            else if (strcmp(key, "polling_times")                    == 0) cfg.polling_times                       = (uint32_t)strtoul(val, nullptr, 10);
            else if (strcmp(key, "color_sensor_cnt")                 == 0) cfg.color_sensor_cnt                    = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "color_sensor_bus_sel")             == 0) parse_u8_array(val, cfg.color_sensor_bus_sel,              COMMS_MAX_SENSORS);
            else if (strcmp(key, "imu_cnt")                          == 0) cfg.imu_cnt                             = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "imu_bus_sel")                      == 0) parse_u8_array(val, cfg.imu_bus_sel,                       COMMS_MAX_SENSORS);
            else if (strcmp(key, "imu_calibration_cnt")              == 0) cfg.imu_calibration_cnt                 = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "distance_sensor_cnt")              == 0) cfg.distance_sensor_cnt                 = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "distance_sensor_bus_sel")          == 0) parse_u8_array(val, cfg.distance_sensor_bus_sel,           COMMS_MAX_SENSORS);
            else if (strcmp(key, "distance_sensor_targeted_dist_mm") == 0) parse_u16_array(val, cfg.distance_sensor_targeted_dist_mm, COMMS_MAX_SENSORS);
            else if (strcmp(key, "distance_sensor_calibration_cnt")  == 0) cfg.distance_sensor_calibration_cnt     = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "io_expander_cnt")                  == 0) cfg.io_expander_cnt                     = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "io_expander_bus_sel")              == 0) parse_u8_array(val, cfg.io_expander_bus_sel,               COMMS_MAX_SENSORS);
            else if (strcmp(key, "io_expander_pin_modes")            == 0) {
                // Flat array of 20 values: [dev0_pin0, dev0_pin1, dev0_pin2, dev0_pin3, dev1_pin0, ...]
                uint8_t flat[COMMS_MAX_SENSORS * 4];
                memset(flat, 255, sizeof(flat));
                parse_u8_array(val, flat, COMMS_MAX_SENSORS * 4);
                for (int d = 0; d < COMMS_MAX_SENSORS; d++)
                    for (int p = 0; p < 4; p++)
                        cfg.io_expander_pin_mode[d][p] = flat[d * 4 + p];
            }
            // unknown keys silently ignored
        }

        line = nl + 1;
    }

    // Clamp polling_times to avoid overrunning the data arrays
    if (cfg.polling_times > 100) cfg.polling_times = 100;

    return true;
}

static size_t cobsEncode(const void * data, size_t length, uint8_t * buffer)
{
  uint8_t * encode = buffer;      // Encoded byte pointer
  uint8_t * codep = encode++;      // Output code pointer
  uint8_t code = 1;       // Code value

  for (const uint8_t * byte = (const uint8_t *)data; length--; ++byte) {
    if (*byte) {           // Byte not zero, write it
      *encode++ = *byte, ++code;
    }

    if (!*byte || (code == 0xff)) {           // Input is zero or block completed, restart
      *codep = code, code = 1, codep = encode;
      if (!*byte || length) {
        ++encode;
      }
    }
  }
  *codep = code;       // Write final code value

  return (size_t)(encode - buffer);
}

/** COBS decode data from buffer
        @param buffer Pointer to encoded input bytes
        @param length Number of bytes to decode
        @param data Pointer to decoded output data
        @return Number of bytes successfully decoded
        @note Stops decoding if delimiter byte is found
 */
static size_t cobsDecode(const uint8_t * buffer, size_t length, void * data)
{
  const uint8_t * byte = buffer;      // Encoded input byte pointer
  uint8_t * decode = (uint8_t *)data;      // Decoded output byte pointer

  for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
    if (block) {           // Decode block byte
      *decode++ = *byte++;
    } else {
      if (code != 0xff) {                 // Encoded zero, write it
        *decode++ = 0;
      }
      block = code = *byte++;                   // Next block length
      if (!code) {                 // Delimiter code found
        break;
      }
    }
  }

  return (size_t)(decode - (uint8_t *)data);
}
