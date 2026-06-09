# Sensor Host ↔ Orin/ROS Serial Protocol

**Status:** DRAFT — for review before implementation.

## 1. Design philosophy

The host (RP2040 "sensor host") is a **dumb I2C bridge**. It knows nothing
about specific sensors. It exposes exactly two primitives over USB serial:

1. **Write** arbitrary bytes to an I2C device on a given port.
2. **Recurring read** — repeatedly (write a pointer string, then read N bytes)
   from a device, every `interval_ms`, `count` times.

All device-specific logic (register maps, init sequences, data decoding) lives
on the **ROS side**. To add a new sensor type you only add code to ROS; the host
firmware never changes.

Each read request carries a **random 2-byte id** chosen by ROS. Every response
echoes that id, so ROS can route results to the right handler without the host
knowing what the bytes mean.

## 2. Transport

- **Link:** USB CDC (virtual serial) on the host. Baud is nominal (USB CDC
  ignores it); use 115200.
- **Framing:** each packet is **COBS-encoded** and terminated by a single
  `0x00` delimiter byte. COBS guarantees the encoded bytes are never `0x00`, so
  the delimiter unambiguously marks frame boundaries and lets a receiver
  resynchronize after any garbage.
- **The host must not emit any non-protocol bytes on this port** (no stray
  `printf`). Debug logging, if any, goes to a separate UART or is compiled out.
- **Byte order:** all multi-byte integer fields are **little-endian**.

### Frame body (before COBS encoding)

```
[ cmd : 1 ][ len : 2 (LE) ][ payload : len ][ checksum : 1 ]
```

- `cmd` — command id (see §3).
- `len` — payload length in bytes (0–`MAX_PAYLOAD`).
- `checksum` — `(cmd + len_lo + len_hi + sum(payload)) & 0xFF`.

On the wire: `COBS(body) , 0x00`.

A receiver accumulates bytes until `0x00`, COBS-decodes, checks the length and
checksum, and drops the frame silently on any mismatch.

`MAX_PAYLOAD` = 512 bytes (proposed).

## 3. Commands

| cmd  | name             | direction      | purpose                          |
|------|------------------|----------------|----------------------------------|
| 0x01 | `ACK`            | host → ROS     | reply to a ROS command           |
| 0x10 | `I2C_WRITE`      | ROS → host     | one-shot write to a device       |
| 0x11 | `READ_REQUEST`   | ROS → host     | start / override / cancel a read |
| 0x12 | `READ_RESULT`    | host → ROS     | one result of a read request     |

ROS sends one command at a time and waits for its `ACK` before sending the next.
`READ_RESULT` messages arrive asynchronously, tagged by id.

### 3.1 `ACK` (0x01) — host → ROS

```
[ ref_cmd : 1 ][ status : 1 ]
```

- `ref_cmd` — the `cmd` this is acknowledging.
- `status` — `0` = OK, non-zero = error (see §5).

### 3.2 `I2C_WRITE` (0x10) — ROS → host

```
[ port : 1 ][ addr : 1 ][ data : N ]
```

- `port` — host port 0–7 (see §4).
- `addr` — 7-bit I2C address.
- `data` — `N = len - 2` bytes written to the device in a single transaction.

Host performs `bus[port].write(addr, data, N)` and replies `ACK(0x10, status)`.

### 3.3 `READ_REQUEST` (0x11) — ROS → host

```
[ id : 2 ][ port : 1 ][ addr : 1 ][ interval_ms : 2 ][ count : 2 ]
[ read_len : 1 ][ write_len : 1 ][ write_bytes : write_len ]
[ post_len : 1 ][ post_bytes : post_len ]
```

- `id` — random 2-byte code chosen by ROS, unique among active reads.
- `port`, `addr` — as above.
- `interval_ms` — spacing between successive reads.
- `count` — number of reads to perform (≥ 1). The read removes itself after the
  last iteration. (No infinite mode; ROS re-issues a request for more.)
  - A `READ_REQUEST` whose `id` matches an already-active read **overrides** it
    (the old read is discarded and replaced with the new parameters).
  - `count = 0` **cancels** the read with that `id` and starts nothing. If the
    `id` was not active it is a harmless no-op. This is the only cancel path.
- `read_len` — bytes to read each time.
- `write_len` / `write_bytes` — bytes written **before** each read (e.g. a
  register pointer). `write_len = 0` means read with no preceding write.
- `post_len` / `post_bytes` — bytes written **after** each read, as a separate
  transaction (e.g. acknowledging a sensor / clearing a data-ready interrupt so
  it produces the next sample). `post_len = 0` means no post-write.

For each iteration the host performs:

```
if write_len > 0: bus[port].write(addr, write_bytes, write_len, nostop=true)
bus[port].read(addr, buf, read_len)
if post_len > 0:  bus[port].write(addr, post_bytes, post_len)
```

then emits one `READ_RESULT`. The pre/post writes let a single recurring read
drive a multi-step sensor handshake entirely on the host — no per-sample
round-trips to ROS. Host replies `ACK(0x11, status)` immediately to confirm the
request was accepted (status non-zero if the table is full or params are
invalid). Up to `MAX_ACTIVE_READS` = 16 reads may run concurrently.

### 3.4 `READ_RESULT` (0x12) — host → ROS

```
[ id : 2 ][ seq : 2 ][ status : 1 ][ data_len : 1 ][ data : data_len ]
```

- `id` — echoes the `READ_REQUEST` id.
- `seq` — iteration index, `0 .. count-1` (free-running for `count = 0`).
- `status` — `0` = OK; non-zero = I2C error, `data_len` then `0`.
- `data` — the bytes read (`data_len == read_len` on success).

## 4. Port and address mapping

Protocol **port 0–7** maps to the host's physical inputs:

| port | input  | SDA / SCL | I2C type        |
|------|--------|-----------|-----------------|
| 0    | INPUT_1| 2 / 3     | hardware (i2c1) |
| 1    | INPUT_2| 4 / 5     | hardware (i2c0) |
| 2    | INPUT_3| 6 / 7     | PIO software    |
| 3    | INPUT_4| 8 / 9     | PIO software    |
| 4    | INPUT_5| 0 / 1     | PIO software    |
| 5    | INPUT_6| 28 / 29   | PIO software    |
| 6    | INPUT_7| 26 / 27   | PIO software    |
| 7    | INPUT_8| 18 / 19   | PIO software    |

`port = input − 1`, so the color sensor on **input 7** is **port 6**.

**I2C address** is set by the device's rotary DAC switch (see
`polling_firmware.h`), which XORs a position-dependent offset onto the device's
default address. On the current build the sensors use switch **0**: the color
sensor (default `0x44`) lands at **`0x3b`** and the distance sensor (default
`0x29`) at **`0x56`**.

## 5. Status codes

| status | meaning                          |
|--------|----------------------------------|
| 0      | OK                               |
| 1      | bad parameters (port/addr/len)   |
| 2      | I2C write failed (NACK/timeout)  |
| 3      | I2C read failed (NACK/timeout)   |
| 4      | read table full / id unknown     |

## 6. Worked example — ISL29125 color sensor

Color sensor on **input 7 (port 6)**, rotary switch **0** → address **`0x3b`**.
A simple register device: no per-read handshake, so `post_len = 0`.

**Initialize** (three one-shot writes; optionally first read reg `0x00`,
expect `0x7D`):

| step | command     | payload bytes                  | effect                |
|------|-------------|--------------------------------|-----------------------|
| 1    | `I2C_WRITE` | `06 3B 01 05`                  | CONFIG1 = 0x05 (RGB)  |
| 2    | `I2C_WRITE` | `06 3B 02 00`                  | CONFIG2 = 0x00        |
| 3    | `I2C_WRITE` | `06 3B 03 00`                  | CONFIG3 = 0x00        |

**Recurring read** of the 6 colour-data bytes (`0x09`..`0x0E`), 20 times at
100 ms:

```
READ_REQUEST:
  id          = <random, e.g. 0xA13F>
  port        = 6
  addr        = 0x3B
  interval_ms = 100
  count       = 20
  read_len    = 6
  write_len   = 1 ; write_bytes = 09     # pointer to GREEN_DATA_LBYTE
  post_len    = 0                        # no post-write needed
```

Each `READ_RESULT.data` is 6 bytes: `G_L G_H R_L R_H B_L B_H`. ROS decodes
`green = G_H<<8 | G_L`, etc., and publishes a `ghost_msgs/ColorSensorState`
on `sensor_host/color_sensor_update`.

## 7. Worked example — VL53L4CD distance sensor

Distance sensor on **input 7 (port 6)**, rotary switch **0** → address **`0x56`**.
A ToF sensor with **16-bit registers** (so register pointers are 2 big-endian
bytes) and a per-sample handshake — the perfect case for `post_bytes`.

**Initialize** — a one-time handshake, driven from ROS (the host stays device
agnostic). Mirrors ST's `VL53L4CD_SensorInit` → `SetRangeTiming(50ms)` →
`StartRanging`: verify model id `0xEBAA` (read reg `0x010F`), wait for boot
(reg `0x00E5 == 0x03`), block-write the 91-byte default config to reg `0x002D`,
run VHV, set the timing budget, then start continuous ranging (`0x0087 = 0x21`).
Reads during init use a one-shot `READ_REQUEST` (`count = 1`).

**Recurring read** of the 15-byte result block at `RESULT__RANGE_STATUS`
(`0x0089`), 20 times at 100 ms, **clearing the data-ready interrupt after each
read** so the sensor advances to the next measurement — entirely on the host:

```
READ_REQUEST:
  id          = <random, e.g. 0xD157>
  port        = 6
  addr        = 0x56
  interval_ms = 100
  count       = 20
  read_len    = 15
  write_len   = 2  ; write_bytes = 00 89    # pointer to RESULT__RANGE_STATUS
  post_len    = 3  ; post_bytes  = 00 86 01 # SYSTEM__INTERRUPT_CLEAR = 0x01
```

Each iteration the host runs `write(00 89) → read(15) → write(00 86 01)`. Each
`READ_RESULT.data` is the 15 result bytes (`0x0089`..`0x0097`); ROS decodes
`range_status = data[0]` (ST remap), `distance_mm = data[13]<<8 | data[14]`,
plus sigma / signal / ambient, and publishes a `ghost_msgs/DistanceSensorState`
on `sensor_host/distance_sensor_update`.

> Because `interval_ms` (100) comfortably exceeds the 50 ms timing budget, the
> measurement triggered by one iteration's post-write is always complete by the
> next read — no data-ready polling needed.

## 8. Out of scope (for now)

- Other sensor types (IMU, IO expander) — same primitives, ROS-side logic
  added later.
- Bus scanning / device discovery.
- Host-side configuration persistence.
