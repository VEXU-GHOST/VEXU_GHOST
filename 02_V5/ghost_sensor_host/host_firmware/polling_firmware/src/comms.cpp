#include "comms.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// COBS
// ---------------------------------------------------------------------------

static size_t cobsEncode(const void *data, size_t length, uint8_t *buffer)
{
  uint8_t *encode = buffer;
  uint8_t *codep = encode++;
  uint8_t code = 1;

  for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte) {
    if (*byte) {
      *encode++ = *byte, ++code;
    }
    if (!*byte || (code == 0xff)) {
      *codep = code, code = 1, codep = encode;
      if (!*byte || length) {
        ++encode;
      }
    }
  }
  *codep = code;
  return (size_t)(encode - buffer);
}

static size_t cobsDecode(const uint8_t *buffer, size_t length, void *data)
{
  const uint8_t *byte = buffer;
  uint8_t *decode = (uint8_t *)data;

  for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
    if (block) {
      *decode++ = *byte++;
    } else {
      block = *byte++;
      if (block && (code != 0xff)) {
        *decode++ = 0;
      }
      code = block;
      if (!code) {
        break;
      }
    }
  }
  return (size_t)(decode - (uint8_t *)data);
}

// ---------------------------------------------------------------------------
// Frame parser
// ---------------------------------------------------------------------------

namespace {
uint8_t  s_cobs_buf[COMMS_MAX_PAYLOAD + 8];
uint16_t s_cobs_len = 0;
uint8_t  s_cmd = 0;
uint16_t s_payload_len = 0;
uint8_t  s_payload[COMMS_MAX_PAYLOAD];
}  // namespace

bool comms_feed_byte(uint8_t b)
{
  if (b != 0x00) {
    if (s_cobs_len < sizeof(s_cobs_buf)) {
      s_cobs_buf[s_cobs_len++] = b;
    } else {
      s_cobs_len = 0;  // overflow, resync
    }
    return false;
  }

  // 0x00 delimiter — decode and validate the framed body.
  bool ready = false;
  if (s_cobs_len > 0) {
    static uint8_t decoded[COMMS_MAX_PAYLOAD + 8];
    size_t dlen = cobsDecode(s_cobs_buf, s_cobs_len, decoded);

    // body: [cmd][len_lo][len_hi][payload][checksum]
    if (dlen >= 4) {
      uint16_t plen = decoded[1] | (static_cast<uint16_t>(decoded[2]) << 8);
      if (plen <= COMMS_MAX_PAYLOAD && dlen == static_cast<size_t>(3 + plen + 1)) {
        uint8_t cs = decoded[0] + decoded[1] + decoded[2];
        for (uint16_t i = 0; i < plen; i++) cs += decoded[3 + i];
        if (cs == decoded[3 + plen]) {
          s_cmd = decoded[0];
          s_payload_len = plen;
          memcpy(s_payload, decoded + 3, plen);
          ready = true;
        }
      }
    }
  }
  s_cobs_len = 0;
  return ready;
}

uint8_t         comms_get_cmd()          { return s_cmd; }
const uint8_t * comms_get_payload()      { return s_payload; }
uint16_t        comms_get_payload_len()  { return s_payload_len; }

void comms_send(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
  static uint8_t body[COMMS_MAX_PAYLOAD + 4];
  uint8_t cs = cmd + static_cast<uint8_t>(len & 0xFF) + static_cast<uint8_t>(len >> 8);
  body[0] = cmd;
  body[1] = static_cast<uint8_t>(len & 0xFF);
  body[2] = static_cast<uint8_t>(len >> 8);
  for (uint16_t i = 0; i < len; i++) {
    body[3 + i] = payload[i];
    cs += payload[i];
  }
  body[3 + len] = cs;

  static uint8_t encoded[COMMS_MAX_PAYLOAD + 8];
  size_t enc_len = cobsEncode(body, 3 + len + 1, encoded);
  for (size_t i = 0; i < enc_len; i++) {
    putchar_raw(encoded[i]);
  }
  putchar_raw(0x00);  // frame delimiter
  fflush(stdout);
}
