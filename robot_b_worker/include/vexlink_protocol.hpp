#pragma once
#include <cstdint> // fixed-width integer types (uint8_t, int16_t, etc.)
#include <cstring> // memcpy

namespace vexlink {

// ── Message types ──────────────────────────────────────────────────────────
// Every packet begins with one of these type bytes so the receiver knows
// which payload struct to decode into.

enum class MsgType : uint8_t {
    ROBOT_STATE = 0x01, // position + heading + status flags (sent over radio)
    COMMAND     = 0x02, // directive sent from manager to worker (sent over radio)
    IMU_DATA    = 0x03, // IMU sensor readings (sent over USB serial to Jetson)
};

// ── Status flag bits ───────────────────────────────────────────────────────
// Packed into the single `flags` byte of RobotStateMsg.
// Use bitwise OR to set multiple flags, bitwise AND to test one.

namespace StateFlags {
    constexpr uint8_t IS_AUTONOMOUS  = 1 << 0; // bit 0: robot is in autonomous mode
    constexpr uint8_t HAS_POSSESSION = 1 << 1; // bit 1: robot is holding a game element
    // bits 2–7 are reserved for future use
}

// ── Payload structs ────────────────────────────────────────────────────────
// __attribute__((packed)) removes any compiler-inserted padding so sizeof()
// is exact and memcpy-based serialization works correctly on both ends.
// Both robots run the same ARM little-endian target, so byte order is consistent.

// Sent by the manager over radio to report its current position and status.
struct __attribute__((packed)) RobotStateMsg {
    int16_t  x_mm;        // x position in millimeters; range ±32 767 mm (≈ ±32 m)
    int16_t  y_mm;        // y position in millimeters
    uint16_t heading_cd;  // heading in centidegrees (0–35 999); divide by 100.0 to get degrees
    uint8_t  flags;       // bit-packed status byte (see StateFlags above)
    // total: 7 bytes
};

// Sent by the manager over radio to instruct the worker to perform an action.
enum class Command : uint8_t {
    STOP       = 0x00, // halt all motion
    GO_TO_GOAL = 0x01, // drive toward the coordinates in param_x / param_y
    INTAKE     = 0x02, // run the intake mechanism
};

struct __attribute__((packed)) CommandMsg {
    uint8_t command;  // which Command to execute
    int16_t param_x;  // optional target x, millimeters (used by GO_TO_GOAL)
    int16_t param_y;  // optional target y, millimeters (used by GO_TO_GOAL)
    // total: 5 bytes
};

// Sent over USB serial to the Jetson Orin Nano for ROS 2 processing.
// The Jetson bridge node converts this into a sensor_msgs/msg/Imu ROS 2 message.
struct __attribute__((packed)) ImuMsg {
    float heading;  // yaw from start orientation, degrees 0–360
    float pitch;    // rotation about the Y axis, degrees; nose up = positive
    float roll;     // rotation about the X axis, degrees; right side down = positive
    float gyro_x;   // angular velocity about the X axis, degrees/second
    float gyro_y;   // angular velocity about the Y axis, degrees/second
    float gyro_z;   // angular velocity about the Z axis, degrees/second
    float accel_x;  // linear acceleration along X, g's; multiply by 9.81 for m/s²
    float accel_y;  // linear acceleration along Y, g's
    float accel_z;  // linear acceleration along Z, g's (includes gravity ≈ 1.0 g at rest)
    // total: 9 × 4 = 36 bytes
};

// ── Radio framing constants ────────────────────────────────────────────────

constexpr size_t HEADER_SIZE = 2;                         // [type : 1 byte][payload_len : 1 byte]
constexpr size_t MAX_PAYLOAD = 64;                        // maximum bytes in a single payload
constexpr size_t MAX_PACKET  = HEADER_SIZE + MAX_PAYLOAD; // maximum total packet size

// ── Serial framing constants ───────────────────────────────────────────────
// USB serial to the Jetson mixes binary packets with ASCII printf output.
// Both sync bytes are > 0x7F, so they are guaranteed not to appear in normal
// ASCII text, letting the Jetson reliably locate packet boundaries.

constexpr uint8_t SERIAL_SYNC_0   = 0xAA;                          // first sync byte
constexpr uint8_t SERIAL_SYNC_1   = 0xBB;                          // second sync byte
constexpr size_t  SERIAL_HDR_SIZE = 4;                             // [0xAA][0xBB][type][len]
constexpr size_t  SERIAL_MAX_PKT  = SERIAL_HDR_SIZE + MAX_PAYLOAD; // maximum serial packet size

// ── encode (radio) ─────────────────────────────────────────────────────────
// Serializes a typed payload struct into `buf` with a 2-byte header prepended.
// Layout: [type][payload_len][payload bytes...]
// Returns the total number of bytes written, or 0 if the buffer is too small.

template <typename T>
inline size_t encode(uint8_t* buf, size_t buf_size, MsgType type, const T& payload) {
    constexpr size_t payload_size = sizeof(T);
    if (HEADER_SIZE + payload_size > buf_size) return 0; // buffer too small; caller must handle
    buf[0] = static_cast<uint8_t>(type);                 // write message type byte
    buf[1] = static_cast<uint8_t>(payload_size);         // write payload length byte
    memcpy(buf + HEADER_SIZE, &payload, payload_size);   // copy struct bytes into buffer after header
    return HEADER_SIZE + payload_size;                   // return total packet size
}

// ── encode_serial (USB serial to Jetson) ───────────────────────────────────
// Same as encode() but prepends two sync bytes (0xAA 0xBB) before the header.
// Layout: [0xAA][0xBB][type][payload_len][payload bytes...]
// The Jetson bridge node scans incoming bytes for the sync sequence to locate
// packet boundaries even when printf output is present on the same port.

template <typename T>
inline size_t encode_serial(uint8_t* buf, size_t buf_size, MsgType type, const T& payload) {
    constexpr size_t payload_size = sizeof(T);
    if (SERIAL_HDR_SIZE + payload_size > buf_size) return 0; // buffer too small
    buf[0] = SERIAL_SYNC_0;                                  // sync byte 0 (0xAA)
    buf[1] = SERIAL_SYNC_1;                                  // sync byte 1 (0xBB)
    buf[2] = static_cast<uint8_t>(type);                     // message type
    buf[3] = static_cast<uint8_t>(payload_size);             // payload length
    memcpy(buf + SERIAL_HDR_SIZE, &payload, payload_size);   // payload bytes
    return SERIAL_HDR_SIZE + payload_size;                   // total bytes written
}

// ── decode_header (radio) ──────────────────────────────────────────────────
// Reads the 2-byte header from a received buffer and validates that enough
// bytes are present for the declared payload length.
// Returns false if the buffer is too short to contain a complete packet.

inline bool decode_header(const uint8_t* buf, size_t buf_len,
                          MsgType& type_out, uint8_t& payload_len_out) {
    if (buf_len < HEADER_SIZE) return false;               // not even a full header yet
    type_out        = static_cast<MsgType>(buf[0]);        // extract the message type byte
    payload_len_out = buf[1];                              // extract the declared payload length
    return buf_len >= HEADER_SIZE + payload_len_out;       // confirm the full payload is present
}

// ── decode_payload ─────────────────────────────────────────────────────────
// Copies the payload bytes (after the 2-byte header) into the output struct.
// `declared_len` must come from a prior call to decode_header.
// Returns false if the declared length is smaller than the target struct (version mismatch).

template <typename T>
inline bool decode_payload(const uint8_t* buf, uint8_t declared_len, T& out) {
    if (sizeof(T) > declared_len) return false;            // struct is larger than what arrived
    memcpy(&out, buf + HEADER_SIZE, sizeof(T));            // copy payload bytes into the output struct
    return true;
}

} // namespace vexlink
