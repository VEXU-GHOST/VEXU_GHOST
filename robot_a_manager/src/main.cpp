/*
 * VEXLink Demo — Robot A (Manager / Sender)
 *
 * Hardware:  V5 Brain + V5 Smart Radio in any Smart Port
 * Framework: PROS 3.x
 *
 * This robot owns the link as the "manager" and periodically
 * transmits a plain string to the partner robot.
 *
 * Shared link ID: both robots MUST use the same id string.
 * Change "my_vexlink" to whatever you chose when pairing
 * the radios in the VEX Device Info screen.
 */

#include "main.h"
#include <cstring>   // memcpy, strlen
#include <cstdio>    // printf

// ── Configuration ──────────────────────────────────────────────────────────

// The Smart Port your radio is plugged into (1–21)
static constexpr uint8_t RADIO_PORT = 11;

// Must exactly match the partner robot's link ID string
static const char * LINK_ID = "my_vexlink";

// How often to send a message (milliseconds)
static constexpr uint32_t SEND_INTERVAL_MS = 500;

// ── Helper: send a null-terminated string ──────────────────────────────────

static constexpr size_t MAX_STR_LEN = 63;

bool send_string(pros::Link& link, const char * msg)
{
  size_t len = strlen(msg);
  if (len > MAX_STR_LEN) len = MAX_STR_LEN;
  uint8_t buf[64];
  buf[0] = static_cast<uint8_t>(len);
  memcpy(buf + 1, msg, len);
  int32_t sent = link.transmit(buf, static_cast<uint16_t>(len + 1));
  return sent != PROS_ERR && sent > 0;
}

// ── PROS entry points ──────────────────────────────────────────────────────

void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Robot A  |  Manager");
  pros::lcd::set_text(1, "Waiting for link...");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol()
{
  static pros::Link tx_link(RADIO_PORT, LINK_ID, pros::E_LINK_TX);
  uint32_t counter = 0;

  while (true) {
    char message[64];
    snprintf(message, sizeof(message), "Hello from A! #%u", counter++);

    bool ok = send_string(tx_link, message);

    // Visual feedback on the V5 brain screen
    pros::lcd::set_text(1, ok ? "TX OK " : "TX FAIL");
    pros::lcd::set_text(2, message);

    // Also print to PROS terminal (visible in `pros terminal`)
    printf("[Manager] %s  →  %s\n", message, ok ? "sent" : "FAILED");

    pros::delay(SEND_INTERVAL_MS);
  }
}
