# ghost_comms

Two standalone PROS projects for bench-testing bidirectional VEXlink radio
communication between V5 Brains, with an opaque 7-byte application payload.
Each Brain also bridges those payloads to a host computer over USB serial.
The manager retains its separate USB-serial IMU stream.

| Project | PROS link role | Radio behavior | Other serial behavior |
| --- | --- | --- | --- |
| `manager` | `pros::E_LINK_TX` | Sends and receives; higher bandwidth role | Sends IMU data and exchanges opaque payloads with its host |
| `worker` | `pros::E_LINK_RX` | Sends and receives; lower bandwidth role | Exchanges opaque payloads with its host |

`E_LINK_TX` and `E_LINK_RX` designate the VEXlink bandwidth roles; they do not
restrict traffic to one direction. PROS reads only the transport headers and
payload length. It does not interpret any of the seven application bytes.

## Layout

```text
ghost_comms/
├── pros_template/             PROS kernel and liblvgl template files
├── common/
│   ├── vexlink_protocol.hpp   Shared fixed-size wire format and parser
│   └── pros_transport.hpp     PROS USB-serial and VEXlink I/O helpers
│   ├── comms_endpoint.hpp     Shared manager/worker loop interface
│   └── comms_endpoint.cpp    Shared serial/radio loop and LCD status
├── manager/                   Higher-bandwidth role and IMU sampling
├── worker/                    Lower-bandwidth role
├── tests/                     Host-buildable C++ protocol tests
└── host_test_client.py        Two-Brain USB serial test client
```

Each project keeps the files PROS considers project-specific. Template files
and the shared protocol header are symlinked into the projects and ignored by
the repository; `pros_template/` stores the shared template once. Each project
also has a source symlink from `src/comms_endpoint.cpp` to the shared
`common/comms_endpoint.cpp`, so PROS compiles the same loop implementation into
both firmware images. The separate `main.cpp` files retain only role setup and
manager IMU handling.

## Wire formats

The application body is always seven arbitrary bytes and is passed unchanged.

| Transport | Frame |
| --- | --- |
| VEXlink | `[type=0x10][length=0x07][7 opaque bytes]` |
| Host to Brain USB serial | `[0xAA][0xBB][type=0x11][length=0x07][7 opaque bytes]` |
| Brain to host USB serial | `[0xAA][0xBB][type=0x12][length=0x07][7 opaque bytes]` |
| Manager IMU USB serial | `[0xAA][0xBB][type=0x03][length=0x24][36 IMU bytes]` |

The host parser resynchronizes on the two sync bytes, rejects host frames with
an unexpected type or length, and ignores non-frame console bytes. Frames that
cannot fit in the eight-entry PROS queue are rejected and counted on the Brain
LCD rather than overwriting queued data.

The firmware uses distinct transmission periods:

| Role | Initial period | Raw radio frame rate | Documented VEXlink limit |
| --- | ---: | ---: | ---: |
| Manager | 20 ms (50 Hz) | 450 B/s | 1040 B/s |
| Worker | 40 ms (25 Hz) | 225 B/s | 520 B/s |

Rates above account for the complete nine-byte application radio frame, not
just the seven-byte body. They are conservative starting points, not guarantees
about throughput. Measure real delivery and loss with the paired radios before
increasing either rate; VEXlink's internal overhead is not included in the
simple application-byte calculation.

This bench protocol is separate from the COBS/checksum protocol used by the
production `ghost_pros` ↔ ROS bridge. Do not connect this host client to a
production bridge expecting that protocol.

## Hardware setup

- Connect a VEXlink radio to manager Smart Port 11 and worker Smart Port 10.
- Connect the manager Brain USB port to one host serial device; connect the
  worker Brain USB port to another.
- The manager also expects an IMU on Smart Port 1.
- Keep `LINK_ID` byte-identical in both programs (`"my_vexlink"` by default).
- Start both programs in operator-control mode; the communication loops run
  from `opcontrol()`. The LCD reports link status, transmit/receive counts,
  queue depth, drops, and errors.

## Build and upload

```sh
bash scripts/pros_build.sh 02_V5/ghost_comms/manager
bash scripts/pros_build.sh 02_V5/ghost_comms/worker
bash scripts/pros_upload.sh 02_V5/ghost_comms/manager
bash scripts/pros_upload.sh 02_V5/ghost_comms/worker
```

The scripts skip the main `ghost_pros` symlink and generated-header steps for
these standalone projects.

## Host bench test

Install `pyserial` in the host Python environment, then connect both Brains and
run:

```sh
python3 -m pip install pyserial
python3 02_V5/ghost_comms/host_test_client.py \
  --manager-port /dev/ttyACM0 \
  --worker-port /dev/ttyACM1
```

The client sends independently generated seven-byte values in both directions
and reports the exact bytes received and round-trip time. The body is printed
as hex only; the client assigns no meaning to its fields. The default injection
period is 100 ms. Use `--duration-s 30` for a bounded run, or
`--interval-ms 80` for a faster test while retaining headroom below the worker's
initial send cadence.

Run the host-side framing checks without hardware:

```sh
python3 02_V5/ghost_comms/host_test_client.py --self-test
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  02_V5/ghost_comms/tests/test_vexlink_protocol.cpp \
  -o /tmp/test_vexlink_protocol
/tmp/test_vexlink_protocol
```

## Maintenance

`pros c apply` can replace symlinks with real files. After a template upgrade,
re-point them:

```sh
cd "$VEXU_HOME/02_V5/ghost_comms"
for project in manager worker; do
  (
    cd "$project"
    ln -sfn ../pros_template/common.mk common.mk
    ln -sfn ../pros_template/firmware firmware
    cd include
    ln -sfn ../../pros_template/include/pros pros
    ln -sfn ../../pros_template/include/liblvgl liblvgl
    ln -sfn ../../pros_template/include/api.h api.h
    ln -sfn ../../common/vexlink_protocol.hpp vexlink_protocol.hpp
    cd ../src
    ln -sfn ../../common/comms_endpoint.cpp comms_endpoint.cpp
  )
done
```

The `templates.*.location` value in each `project.pros` is machine-local and
is used only by `pros c upgrade`; do not commit local path changes.

Run the repository's formatter before a PR:

```sh
uncrustify -c uncrustify.cfg --no-backup \
  02_V5/ghost_comms/*/src/*.cpp \
  02_V5/ghost_comms/common/*.hpp
```
