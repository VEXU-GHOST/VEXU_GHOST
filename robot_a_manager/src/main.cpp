#include "main.h"              // PROS framework entry points and core headers
#include "vexlink_protocol.hpp" // shared packet definitions
#include <cstdio>               // printf, fwrite, fflush

// ── Configuration ──────────────────────────────────────────────────────────

static constexpr uint8_t  RADIO_PORT    = 11; // Smart Port the radio is plugged into
static constexpr uint8_t  IMU_PORT      = 1;  // Smart Port the IMU is plugged into — update to match hardware
static const char*        LINK_ID       = "my_vexlink"; // must match the worker's LINK_ID
static constexpr uint32_t IMU_INTERVAL  = 20; // ms per loop tick — 50 Hz IMU serial send rate
static constexpr uint32_t RADIO_EVERY_N = 10; // send radio once every N ticks (= 200 ms)

// File-scope IMU object so both initialize() and opcontrol() share the same instance.
// pros::IMU constructor is non-blocking (unlike pros::Link), so global scope is safe here.
static pros::IMU imu(IMU_PORT);

// ── serial_send ────────────────────────────────────────────────────────────
// Writes raw bytes to the USB serial port (stdout in PROS).
// The Jetson bridge node reads this same port at /dev/ttyACM0.

void serial_send(const uint8_t* buf, size_t len)
{
    fwrite(buf, 1, len, stdout); // stdout routes to USB serial in PROS
    fflush(stdout);              // flush immediately so the Jetson receives complete packets
}

// ── send_imu_serial ────────────────────────────────────────────────────────
// Reads all IMU fields, encodes them into an ImuMsg packet with sync bytes,
// and sends it over USB serial to the Jetson Orin Nano.

void send_imu_serial()
{
    vexlink::ImuMsg msg;

    msg.heading = static_cast<float>(imu.get_heading()); // 0–360 degrees (yaw from start)

    pros::euler_s_t euler = imu.get_euler();     // pitch, roll, yaw in degrees
    msg.pitch = static_cast<float>(euler.pitch); // nose up = positive
    msg.roll  = static_cast<float>(euler.roll);  // right side down = positive

    pros::imu_gyro_s_t gyro = imu.get_gyro_rate(); // angular velocity in degrees/second
    msg.gyro_x = static_cast<float>(gyro.x);
    msg.gyro_y = static_cast<float>(gyro.y);
    msg.gyro_z = static_cast<float>(gyro.z);

    pros::imu_accel_s_t accel = imu.get_accel(); // linear acceleration in g's (1 g = 9.81 m/s²)
    msg.accel_x = static_cast<float>(accel.x);
    msg.accel_y = static_cast<float>(accel.y);
    msg.accel_z = static_cast<float>(accel.z);   // expect ≈ 1.0 g at rest due to gravity

    uint8_t buf[vexlink::SERIAL_HDR_SIZE + sizeof(msg)];
    size_t  len = vexlink::encode_serial(buf, sizeof(buf), vexlink::MsgType::IMU_DATA, msg);
    if (len > 0) serial_send(buf, len); // only send if encode succeeded
}

// ── send_state ─────────────────────────────────────────────────────────────
// Encodes a RobotStateMsg and transmits it over VEXLink radio to the worker robot.
// Returns true if the radio accepted the bytes, false on error.

bool send_state(pros::Link& link, int16_t x_mm, int16_t y_mm,
                uint16_t heading_cd, uint8_t flags)
{
    vexlink::RobotStateMsg payload{ x_mm, y_mm, heading_cd, flags }; // fill the struct
    uint8_t buf[vexlink::HEADER_SIZE + sizeof(payload)];
    size_t  len = vexlink::encode(buf, sizeof(buf), vexlink::MsgType::ROBOT_STATE, payload);
    if (len == 0) return false;
    uint32_t sent = link.transmit_raw(buf, static_cast<uint16_t>(len));
    return sent != static_cast<uint32_t>(PROS_ERR) && sent > 0;
}

// ── PROS entry points ──────────────────────────────────────────────────────

// initialize() runs once at power-on before any competition mode starts
void initialize()
{
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Robot A  |  Manager");

    // Trigger IMU calibration — takes roughly 2 seconds; robot must remain still
    imu.reset();
    pros::lcd::set_text(1, "IMU calibrating...");
    while (imu.is_calibrating()) {
        pros::delay(50); // poll every 50 ms until the IMU reports it is done
    }
    pros::lcd::set_text(1, "IMU ready");
}

void disabled() {}             // called when robot is disabled by field controller (nothing to do)
void competition_initialize() {} // called before autonomous while still disabled (nothing to do)
void autonomous() {}           // autonomous period (not used on this robot)

// opcontrol() is the main driver-control loop; runs until the match ends
void opcontrol()
{
    // Static so the Link object is constructed once — avoids the global-scope blocking issue
    static pros::Link tx_link(RADIO_PORT, LINK_ID, pros::E_LINK_TX);

    // Simulated position — replace with real GPS / odometry reads later
    int16_t  x_mm       = 0;
    int16_t  y_mm       = 500;
    uint16_t heading_cd = 0;
    uint32_t cycle      = 0; // counts loop ticks, used to throttle the radio send

    while (true) {
        // ── IMU serial send — every tick (50 Hz) ──────────────────────────
        send_imu_serial(); // read IMU and send over USB serial to Jetson

        // ── Radio send — every RADIO_EVERY_N ticks (200 ms) ───────────────
        if (cycle % RADIO_EVERY_N == 0) {
            // Advance simulated values so the worker screen shows visible change
            x_mm       = (x_mm + 50) % 3600;         // cycle x across 0–3599 mm
            heading_cd = (heading_cd + 500) % 36000;  // rotate heading across 0–359.99°

            uint8_t flags = pros::competition::is_autonomous()
                            ? vexlink::StateFlags::IS_AUTONOMOUS : 0; // set autonomous flag bit

            bool ok = send_state(tx_link, x_mm, y_mm, heading_cd, flags);

            char line[64];
            snprintf(line, sizeof(line), "X:%d Hdg:%.1f", x_mm, heading_cd / 100.0f);
            pros::lcd::set_text(1, ok ? "TX OK" : "TX FAIL"); // radio status
            pros::lcd::set_text(2, line);                      // current simulated position
        }

        ++cycle;
        pros::delay(IMU_INTERVAL); // wait 20 ms before next tick (50 Hz loop rate)
    }
}
