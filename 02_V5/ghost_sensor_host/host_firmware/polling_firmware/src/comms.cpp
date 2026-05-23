#include "comms.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Packet parser
// ---------------------------------------------------------------------------

namespace {

enum class ParseState : uint8_t {
    WAIT_MAGIC0,
    WAIT_MAGIC1,
    WAIT_CMD,
    WAIT_LEN_LO,
    WAIT_LEN_HI,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM,
};

static ParseState  s_state        = ParseState::WAIT_MAGIC0;
static uint8_t     s_cmd          = 0;
static uint16_t    s_len          = 0;
static uint16_t    s_rx_count     = 0;
static uint8_t     s_checksum     = 0;
static uint8_t     s_payload[COMMS_MAX_PAYLOAD];
static bool        s_packet_ready = false;

} // namespace

bool comms_feed_byte(uint8_t b) {
    s_packet_ready = false;

    switch (s_state) {
        case ParseState::WAIT_MAGIC0:
            if (b == COMMS_MAGIC_0) s_state = ParseState::WAIT_MAGIC1;
            break;

        case ParseState::WAIT_MAGIC1:
            s_state = (b == COMMS_MAGIC_1) ? ParseState::WAIT_CMD
                                            : ParseState::WAIT_MAGIC0;
            break;

        case ParseState::WAIT_CMD:
            s_cmd      = b;
            s_checksum = b;
            s_state    = ParseState::WAIT_LEN_LO;
            break;

        case ParseState::WAIT_LEN_LO:
            s_len      = b;
            s_checksum ^= b;
            s_state    = ParseState::WAIT_LEN_HI;
            break;

        case ParseState::WAIT_LEN_HI:
            s_len      |= (static_cast<uint16_t>(b) << 8);
            s_checksum ^= b;
            if (s_len > COMMS_MAX_PAYLOAD) {
                s_state = ParseState::WAIT_MAGIC0;  // reject oversized packet
            } else if (s_len == 0) {
                s_state = ParseState::WAIT_CHECKSUM;
            } else {
                s_rx_count = 0;
                s_state    = ParseState::WAIT_PAYLOAD;
            }
            break;

        case ParseState::WAIT_PAYLOAD:
            s_payload[s_rx_count++] = b;
            s_checksum ^= b;
            if (s_rx_count == s_len) {
                s_state = ParseState::WAIT_CHECKSUM;
            }
            break;

        case ParseState::WAIT_CHECKSUM:
            s_state = ParseState::WAIT_MAGIC0;
            if (b == s_checksum) {
                s_packet_ready = true;
            }
            break;
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
    uint8_t cs = cmd ^ (uint8_t)(len & 0xFF) ^ (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) cs ^= payload[i];

    putchar_raw(COMMS_MAGIC_0);
    putchar_raw(COMMS_MAGIC_1);
    putchar_raw(cmd);
    putchar_raw((uint8_t)(len & 0xFF));
    putchar_raw((uint8_t)(len >> 8));
    for (uint16_t i = 0; i < len; i++) putchar_raw(payload[i]);
    putchar_raw(cs);
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

            if      (strcmp(key, "polling_interval_ms")    == 0) cfg.polling_interval_ms    = (uint32_t)strtoul(val, nullptr, 10);
            else if (strcmp(key, "polling_times")          == 0) cfg.polling_times          = (uint32_t)strtoul(val, nullptr, 10);
            else if (strcmp(key, "color_sensor_cnt")       == 0) cfg.color_sensor_cnt       = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "color_sensor_bus_sel")   == 0) parse_u8_array(val, cfg.color_sensor_bus_sel,    COMMS_MAX_SENSORS);
            else if (strcmp(key, "imu_cnt")                == 0) cfg.imu_cnt                = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "imu_bus_sel")            == 0) parse_u8_array(val, cfg.imu_bus_sel,             COMMS_MAX_SENSORS);
            else if (strcmp(key, "distance_sensor_cnt")    == 0) cfg.distance_sensor_cnt    = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "distance_sensor_bus_sel")== 0) parse_u8_array(val, cfg.distance_sensor_bus_sel, COMMS_MAX_SENSORS);
            else if (strcmp(key, "io_expander_cnt")        == 0) cfg.io_expander_cnt        = (uint8_t) strtoul(val, nullptr, 10);
            else if (strcmp(key, "io_expander_bus_sel")    == 0) parse_u8_array(val, cfg.io_expander_bus_sel,     COMMS_MAX_SENSORS);
            else if (strcmp(key, "io_expander_pin_modes")  == 0) {
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
