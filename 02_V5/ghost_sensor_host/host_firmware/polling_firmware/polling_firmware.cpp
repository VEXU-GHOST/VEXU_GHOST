// ---------------------------------------------------------------------------
// Sensor Host firmware — generic I2C bridge.
//
// The host knows nothing about specific sensors. It exposes two operations over
// USB serial (see ghost_sensor_host/PROTOCOL.md):
//   CMD_I2C_WRITE     write arbitrary bytes to a device
//   CMD_READ_REQUEST  recurring "write pointer then read N bytes", count times,
//                     every interval_ms; each result tagged with the request id
// All device-specific logic lives on the ROS side.
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "polling_firmware.h"     // pin map, I2C_FREQ_HZ
#include "comms.h"
#include "I2CBus.h"
#include "SoftwareI2CBus.h"
#include "HardwareI2CBus.h"
#include "i2c.pio.h"

#define NUM_PORTS         8
#define MAX_ACTIVE_READS  16
#define MAX_WRITE_BYTES   32
#define MAX_POST_BYTES    16
#define MAX_READ_BYTES    64

// ---------------------------------------------------------------------------
// I2C buses — index = protocol port (0..7) -> physical input 1..8
// ---------------------------------------------------------------------------

static I2CBus *g_bus[NUM_PORTS];
static uint8_t g_sda[NUM_PORTS];
static uint8_t g_scl[NUM_PORTS];

static void init_i2c_buses()
{
  uint off0 = pio_add_program(pio0, &i2c_program);
  uint off1 = pio_add_program(pio1, &i2c_program);

  // Ports 0,1 = hardware I2C; ports 2..7 = PIO software I2C.
  g_bus[0] = new HardwareI2CBus(i2c1);
  g_bus[1] = new HardwareI2CBus(i2c0);
  g_bus[2] = new SoftwareI2CBus(3, pio0, 0, off0, SDA_INPUT_3, SCL_INPUT_3, I2C_FREQ_HZ);
  g_bus[3] = new SoftwareI2CBus(4, pio0, 1, off0, SDA_INPUT_4, SCL_INPUT_4, I2C_FREQ_HZ);
  g_bus[4] = new SoftwareI2CBus(5, pio0, 2, off0, SDA_INPUT_5, SCL_INPUT_5, I2C_FREQ_HZ);
  g_bus[5] = new SoftwareI2CBus(6, pio1, 0, off1, SDA_INPUT_6, SCL_INPUT_6, I2C_FREQ_HZ);
  g_bus[6] = new SoftwareI2CBus(7, pio1, 1, off1, SDA_INPUT_7, SCL_INPUT_7, I2C_FREQ_HZ);
  g_bus[7] = new SoftwareI2CBus(8, pio1, 2, off1, SDA_INPUT_8, SCL_INPUT_8, I2C_FREQ_HZ);

  g_sda[0] = SDA_INPUT_1; g_scl[0] = SCL_INPUT_1;
  g_sda[1] = SDA_INPUT_2; g_scl[1] = SCL_INPUT_2;
  g_sda[2] = SDA_INPUT_3; g_scl[2] = SCL_INPUT_3;
  g_sda[3] = SDA_INPUT_4; g_scl[3] = SCL_INPUT_4;
  g_sda[4] = SDA_INPUT_5; g_scl[4] = SCL_INPUT_5;
  g_sda[5] = SDA_INPUT_6; g_scl[5] = SCL_INPUT_6;
  g_sda[6] = SDA_INPUT_7; g_scl[6] = SCL_INPUT_7;
  g_sda[7] = SDA_INPUT_8; g_scl[7] = SCL_INPUT_8;

  // Hardware buses need the SDK peripheral + pad setup; PIO buses self-configure.
  for (int p = 0; p < 2; p++) {
    i2c_init(g_bus[p]->get_i2c_instance(), I2C_FREQ_HZ);
    gpio_set_function(g_sda[p], GPIO_FUNC_I2C);
    gpio_set_function(g_scl[p], GPIO_FUNC_I2C);
    gpio_pull_up(g_sda[p]);
    gpio_pull_up(g_scl[p]);
  }
}

// ---------------------------------------------------------------------------
// Active recurring reads
// ---------------------------------------------------------------------------

struct ActiveRead {
  bool     active;
  uint16_t id;
  uint8_t  port;
  uint8_t  addr;
  uint16_t interval_ms;
  uint8_t  read_len;
  uint8_t  write_len;
  uint8_t  write_bytes[MAX_WRITE_BYTES];
  uint8_t  post_len;
  uint8_t  post_bytes[MAX_POST_BYTES];
  uint16_t seq;
  uint16_t remaining;
  uint32_t next_fire_ms;
};

static ActiveRead g_reads[MAX_ACTIVE_READS];

static inline uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static void send_ack(uint8_t ref_cmd, uint8_t status)
{
  uint8_t p[2] = {ref_cmd, status};
  comms_send(CMD_ACK, p, 2);
}

static void handle_i2c_write(const uint8_t *p, uint16_t len)
{
  if (len < 2) { send_ack(CMD_I2C_WRITE, ST_BAD_PARAMS); return; }
  uint8_t port = p[0];
  uint8_t addr = p[1];
  if (port >= NUM_PORTS) { send_ack(CMD_I2C_WRITE, ST_BAD_PARAMS); return; }
  int r = g_bus[port]->write(addr, p + 2, len - 2);
  send_ack(CMD_I2C_WRITE, (r < 0) ? ST_WRITE_FAIL : ST_OK);
}

// payload: [id:2][port][addr][interval:2][count:2][read_len][write_len][write_bytes]
//          [post_len][post_bytes]
static void handle_read_request(const uint8_t *p, uint16_t len)
{
  if (len < 10) { send_ack(CMD_READ_REQUEST, ST_BAD_PARAMS); return; }
  uint16_t id       = p[0] | (uint16_t(p[1]) << 8);
  uint8_t  port     = p[2];
  uint8_t  addr     = p[3];
  uint16_t interval = p[4] | (uint16_t(p[5]) << 8);
  uint16_t count    = p[6] | (uint16_t(p[7]) << 8);
  uint8_t  read_len = p[8];
  uint8_t  write_len = p[9];

  // post-write follows the write_bytes: [post_len][post_bytes]
  if (len < uint16_t(11 + write_len)) { send_ack(CMD_READ_REQUEST, ST_BAD_PARAMS); return; }
  uint8_t  post_len = p[10 + write_len];

  if (port >= NUM_PORTS || read_len > MAX_READ_BYTES ||
      write_len > MAX_WRITE_BYTES || post_len > MAX_POST_BYTES ||
      len != uint16_t(11 + write_len + post_len)) {
    send_ack(CMD_READ_REQUEST, ST_BAD_PARAMS);
    return;
  }

  // Find an existing read with this id (override/cancel) and the first free slot.
  int slot = -1, free_slot = -1;
  for (int i = 0; i < MAX_ACTIVE_READS; i++) {
    if (g_reads[i].active && g_reads[i].id == id) { slot = i; break; }
    if (!g_reads[i].active && free_slot < 0) free_slot = i;
  }

  if (count == 0) {                      // cancel
    if (slot >= 0) g_reads[slot].active = false;
    send_ack(CMD_READ_REQUEST, ST_OK);
    return;
  }

  if (slot < 0) slot = free_slot;
  if (slot < 0) { send_ack(CMD_READ_REQUEST, ST_TABLE_FULL); return; }

  ActiveRead &r = g_reads[slot];
  r.active = true;
  r.id = id;
  r.port = port;
  r.addr = addr;
  r.interval_ms = interval;
  r.read_len = read_len;
  r.write_len = write_len;
  memcpy(r.write_bytes, p + 10, write_len);
  r.post_len = post_len;
  memcpy(r.post_bytes, p + 11 + write_len, post_len);
  r.seq = 0;
  r.remaining = count;
  r.next_fire_ms = now_ms();
  send_ack(CMD_READ_REQUEST, ST_OK);
}

static void dispatch(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
  switch (cmd) {
    case CMD_I2C_WRITE:    handle_i2c_write(payload, len);    break;
    case CMD_READ_REQUEST: handle_read_request(payload, len); break;
    default: /* unknown — ignore */ break;
  }
}

// Perform any reads that are due, emit a READ_RESULT for each.
static void service_reads()
{
  uint32_t now = now_ms();
  for (int i = 0; i < MAX_ACTIVE_READS; i++) {
    ActiveRead &r = g_reads[i];
    if (!r.active) continue;
    if (int32_t(now - r.next_fire_ms) < 0) continue;

    uint8_t status = ST_OK;
    uint8_t data[MAX_READ_BYTES];

    if (r.write_len > 0) {
      int wr = g_bus[r.port]->write(r.addr, r.write_bytes, r.write_len, /*nostop=*/true);
      if (wr < 0) status = ST_WRITE_FAIL;
    }
    if (status == ST_OK) {
      int rd = g_bus[r.port]->read(r.addr, data, r.read_len);
      if (rd < 0) status = ST_READ_FAIL;
    }
    // Post-write (e.g. clear a data-ready interrupt) so the device advances to
    // its next sample before we read again. Sent regardless of read status.
    if (r.post_len > 0) {
      g_bus[r.port]->write(r.addr, r.post_bytes, r.post_len);
    }

    // READ_RESULT: [id:2][seq:2][status:1][data_len:1][data]
    uint8_t out[6 + MAX_READ_BYTES];
    uint8_t dlen = (status == ST_OK) ? r.read_len : 0;
    out[0] = r.id & 0xFF;  out[1] = r.id >> 8;
    out[2] = r.seq & 0xFF; out[3] = r.seq >> 8;
    out[4] = status;
    out[5] = dlen;
    if (dlen) memcpy(out + 6, data, dlen);
    comms_send(CMD_READ_RESULT, out, 6 + dlen);

    r.seq++;
    r.remaining--;
    r.next_fire_ms += r.interval_ms;
    if (r.remaining == 0) r.active = false;
  }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
  stdio_init_all();
  sleep_ms(500);
  init_i2c_buses();
  memset(g_reads, 0, sizeof(g_reads));

  while (true) {
    // Drain all available input bytes through the frame parser.
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
      if (comms_feed_byte((uint8_t)c)) {
        dispatch(comms_get_cmd(), comms_get_payload(), comms_get_payload_len());
      }
    }

    service_reads();
    sleep_us(200);
  }
}
