#include "main.h"              // PROS framework entry points and core headers
#include "vexlink_protocol.hpp" // shared packet definitions (MsgType, RobotStateMsg, encode/decode)
#include <cstdio>               // printf

// ── Configuration ──────────────────────────────────────────────────────────

static constexpr uint8_t  RADIO_PORT    = 10;           // Smart Port the radio is plugged into
static const char*        LINK_ID       = "my_vexlink"; // must exactly match the manager's LINK_ID
static constexpr uint32_t POLL_INTERVAL = 50;           // how often to check for new packets (ms)

// ── receive_state ──────────────────────────────────────────────────────────
// Checks the radio receive buffer for a complete ROBOT_STATE packet.
// Returns true and writes the decoded state into `out` if a valid packet was found.
// Returns false if there is no data, not enough data, or the packet type doesn't match.

bool receive_state(pros::Link& link, vexlink::RobotStateMsg& out)
{
    uint32_t avail = link.raw_receivable_size(); // number of raw bytes waiting in the receive buffer

    // Need at least a full header plus one payload byte before bothering to read
    if (avail < vexlink::HEADER_SIZE + 1) return false;

    uint8_t  buf[vexlink::MAX_PACKET];                                                    // scratch buffer large enough for any valid packet
    uint16_t to_read = static_cast<uint16_t>(avail < sizeof(buf) ? avail : sizeof(buf)); // read all available bytes, capped to buffer size
    int32_t  got     = static_cast<int32_t>(link.receive_raw(buf, to_read));             // pull bytes out of the radio FIFO

    // receive_raw returns PROS_ERR on failure, or the number of bytes actually read
    if (got == PROS_ERR || static_cast<size_t>(got) < vexlink::HEADER_SIZE) return false;

    vexlink::MsgType type;       // will hold the message type byte from the header
    uint8_t          payload_len; // will hold the declared payload length from the header
    // decode_header validates the buffer is long enough and extracts type + length
    if (!vexlink::decode_header(buf, static_cast<size_t>(got), type, payload_len)) return false;

    // Ignore packets that aren't the type we're expecting
    if (type != vexlink::MsgType::ROBOT_STATE) return false;

    // Copy the payload bytes into the output struct and return success
    return vexlink::decode_payload(buf, payload_len, out);
}

// ── PROS entry points ──────────────────────────────────────────────────────

// initialize() runs once at power-on before any competition mode starts
void initialize()
{
    pros::lcd::initialize();                    // turn on the brain's LCD display
    pros::lcd::set_text(0, "Robot B  |  Partner"); // line 0: static label
    pros::lcd::set_text(1, "Waiting...");           // line 1: placeholder until link is established
}

void disabled() {}             // called when robot is disabled by field controller (nothing to do)
void competition_initialize() {} // called before autonomous while still disabled (nothing to do)
void autonomous() {}           // autonomous period (not used on this robot)

// opcontrol() is the main driver-control loop; runs until the match ends
void opcontrol()
{
    // Declared static so the Link object is constructed once and persists across calls.
    // Constructing pros::Link at global scope would block LCD init, so we use static here.
    static pros::Link rx_link(RADIO_PORT, LINK_ID, pros::E_LINK_RX);

    vexlink::RobotStateMsg last_state{}; // most recently received state, zero-initialized
    bool has_state = false;              // tracks whether we have received at least one valid packet

    while (true) {
        vexlink::RobotStateMsg state{}; // temporary storage for the incoming packet

        if (receive_state(rx_link, state)) {
            last_state = state;  // update the persistent copy with the fresh data
            has_state  = true;   // mark that we now have at least one good reading
            printf("[Partner] x=%d y=%d hdg=%.1f flags=%02x\n",
                   state.x_mm, state.y_mm, state.heading_cd / 100.0f, state.flags); // log to PROS terminal
        }

        if (has_state) {
            // At least one packet received — display the most recent state on the brain screen
            char pos[64], hdg[64];
            snprintf(pos, sizeof(pos), "X:%d Y:%d mm", last_state.x_mm, last_state.y_mm); // format position string
            snprintf(hdg, sizeof(hdg), "Hdg:%.1f  %s",
                     last_state.heading_cd / 100.0f,                                           // convert centidegrees → degrees
                     (last_state.flags & vexlink::StateFlags::IS_AUTONOMOUS) ? "[AUTO]" : "[DRIVER]"); // decode the autonomous flag bit
            pros::lcd::set_text(1, pos); // show position on line 1
            pros::lcd::set_text(2, hdg); // show heading + mode on line 2
        } else {
            // No packet yet — show radio link status so we know the connection state
            pros::lcd::set_text(1, pros::c::link_connected(RADIO_PORT) ? "Linked" : "NOT LINKED");
        }

        pros::delay(POLL_INTERVAL); // yield for POLL_INTERVAL ms before checking again
    }
}
