/*
 * VEXLink Demo — Robot B (Partner / Receiver)
 *
 * Hardware:  V5 Brain + V5 Smart Radio in any Smart Port
 * Framework: PROS 3.x
 *
 * This robot joins the link as the "partner" and prints every
 * string it receives from the manager robot.
 *
 * Shared link ID: both robots MUST use the same id string.
 * Change "my_vexlink" to whatever you chose when pairing
 * the radios in the VEX Device Info screen.
 */

#include "main.h"
#include <cstring>   // memcpy
#include <cstdio>    // printf

// ── Configuration ──────────────────────────────────────────────────────────

static constexpr uint8_t RADIO_PORT = 10;
static const char * LINK_ID = "my_vexlink";           // must match Robot A

// How often to poll for incoming data (milliseconds)
static constexpr uint32_t POLL_INTERVAL_MS = 50;

// ── Helper: receive a string (non-blocking) ────────────────────────────────

static constexpr size_t MAX_STR_LEN = 63;

bool receive_string(pros::Link& link, char * out_buf, size_t out_size)
{
  uint8_t raw[64] = {};
  int32_t got = link.receive(raw, sizeof(raw));

  uint32_t raw_avail = link.raw_receivable_size();
  char dbg[48];
  snprintf(dbg, sizeof(dbg), "got=%d raw=%u", (int)got, raw_avail);
  pros::lcd::set_text(3, dbg);

  if (got == PROS_ERR || got < 2) return false;

  size_t len = raw[0];
  if (len >= out_size) len = out_size - 1;
  memcpy(out_buf, raw + 1, len);
  out_buf[len] = '\0';
  return true;
}

// ── PROS entry points ──────────────────────────────────────────────────────

void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Robot B  |  Partner");
  pros::lcd::set_text(1, "Waiting for link...");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol()
{
  static pros::Link rx_link(RADIO_PORT, LINK_ID, pros::E_LINK_RX);
  char last_msg[64] = "(none yet)";

  while (true) {
    char incoming[64];

    bool linked = pros::c::link_connected(RADIO_PORT);
    if (receive_string(rx_link, incoming, sizeof(incoming))) {
      strncpy(last_msg, incoming, sizeof(last_msg) - 1);
      last_msg[sizeof(last_msg) - 1] = '\0';
      pros::lcd::set_text(1, "RX OK");
      pros::lcd::set_text(2, last_msg);
      printf("[Partner] received: \"%s\"\n", last_msg);
    } else {
      pros::lcd::set_text(1, linked ? "Linked, no data" : "NOT LINKED");
      pros::lcd::set_text(2, last_msg);
    }

    pros::delay(POLL_INTERVAL_MS);
  }
}
