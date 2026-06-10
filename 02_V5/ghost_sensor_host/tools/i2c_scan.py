#!/usr/bin/env python3
"""I2C bus scanner for the ghost_sensor_host RP2040 board.

Speaks the COBS serial protocol in ../PROTOCOL.md and probes every I2C address
on every host port, reporting which devices respond. Each address is probed
with a single 1-byte READ_REQUEST (count=1); a READ_RESULT status of OK means
the device ACKed its address (present), READ_FAIL means it NACKed (absent).
The 1-byte read works as a presence probe on both the hardware (ports 0-1) and
the PIO software buses; a zero-length write does NOT (the Pico SDK skips the
transaction for len==0 on the hardware buses), so the read probe is the default.

Usage:
    python3 i2c_scan.py                         # autodetect port, scan all ports
    python3 i2c_scan.py --port /dev/ttyACM0
    python3 i2c_scan.py --ports 6,7             # only host ports 6 and 7
    python3 i2c_scan.py --start 0x08 --end 0x77
    python3 i2c_scan.py --write-probe           # zero-length write probe (PIO ports only)

Requires pyserial (`pip install pyserial`).
"""

import argparse
import glob
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("error: pyserial not installed. Run: pip install pyserial")

# ---------------------------------------------------------------------------
# Protocol constants (see PROTOCOL.md / polling_firmware/inc/comms.h)
# ---------------------------------------------------------------------------

CMD_ACK = 0x01
CMD_I2C_WRITE = 0x10
CMD_READ_REQUEST = 0x11
CMD_READ_RESULT = 0x12

ST_OK = 0
ST_BAD_PARAMS = 1
ST_WRITE_FAIL = 2
ST_READ_FAIL = 3
ST_TABLE_FULL = 4

NUM_PORTS = 8

# port -> (physical input label, bus kind), from PROTOCOL.md §4
PORT_INFO = {
    0: ("INPUT_1", "hw i2c1"),
    1: ("INPUT_2", "hw i2c0"),
    2: ("INPUT_3", "PIO sw"),
    3: ("INPUT_4", "PIO sw"),
    4: ("INPUT_5", "PIO sw"),
    5: ("INPUT_6", "PIO sw"),
    6: ("INPUT_7", "PIO sw"),
    7: ("INPUT_8", "PIO sw"),
}

# Known device defaults + rotary-DAC offsets, for annotating discovered
# addresses (PROTOCOL.md §8.1). address = DEFAULT_ADDR[type] ^ DAC_OFFSET[sw]
DEFAULT_ADDR = {"COLOR": 0x44, "DISTANCE": 0x29, "IMU": 0x68, "IO_EXPANDER": 0x41}
DAC_OFFSET = [
    0x7F, 0x75, 0x7A, 0x70, 0x2F, 0x25, 0x2A, 0x20,
    0x4F, 0x45, 0x4A, 0x40, 0x0F, 0x05, 0x0A, 0x00,
]


def identify(addr):
    """Return a list of "(TYPE, switch N)" device guesses for an I2C address."""
    out = []
    for dev_type, default in DEFAULT_ADDR.items():
        for sw, off in enumerate(DAC_OFFSET):
            if (default ^ off) == addr:
                out.append(f"{dev_type} sw{sw}")
    return out


# ---------------------------------------------------------------------------
# COBS (matches cobsEncode/cobsDecode in comms.cpp)
# ---------------------------------------------------------------------------

def cobs_encode(data):
    out = bytearray()
    code_pos = len(out)
    out.append(0)  # placeholder for the first code byte
    code = 1
    for b in data:
        if b == 0:
            out[code_pos] = code
            code_pos = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_pos] = code
                code_pos = len(out)
                out.append(0)
                code = 1
    out[code_pos] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    idx = 0
    n = len(data)
    while idx < n:
        code = data[idx]
        idx += 1
        if code == 0:
            break
        for _ in range(code - 1):
            if idx >= n:
                break
            out.append(data[idx])
            idx += 1
        if code < 0xFF and idx < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# Frame build / parse
# ---------------------------------------------------------------------------

def build_frame(cmd, payload):
    """Return COBS(body) + 0x00 for one command frame."""
    length = len(payload)
    body = bytearray([cmd, length & 0xFF, (length >> 8) & 0xFF])
    body.extend(payload)
    cs = (cmd + (length & 0xFF) + ((length >> 8) & 0xFF) + sum(payload)) & 0xFF
    body.append(cs)
    return cobs_encode(bytes(body)) + b"\x00"


def parse_body(decoded):
    """Validate a COBS-decoded body. Return (cmd, payload) or None."""
    if len(decoded) < 4:
        return None
    cmd = decoded[0]
    plen = decoded[1] | (decoded[2] << 8)
    if len(decoded) != 3 + plen + 1:
        return None
    payload = decoded[3:3 + plen]
    cs = (cmd + decoded[1] + decoded[2] + sum(payload)) & 0xFF
    if cs != decoded[3 + plen]:
        return None
    return cmd, bytes(payload)


class FrameReader:
    """Accumulates serial bytes and yields validated (cmd, payload) frames."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()

    def poll(self, deadline):
        """Yield frames seen before `deadline` (monotonic seconds)."""
        while time.monotonic() < deadline:
            n = self.ser.in_waiting or 1
            chunk = self.ser.read(n)
            if not chunk:
                continue
            for b in chunk:
                if b == 0x00:
                    if self.buf:
                        frame = parse_body(cobs_decode(bytes(self.buf)))
                        self.buf.clear()
                        if frame:
                            yield frame
                else:
                    self.buf.append(b)


# ---------------------------------------------------------------------------
# Probes
# ---------------------------------------------------------------------------

def read_request_payload(req_id, port, addr):
    """A count=1, 1-byte read with no pre/post write — a pure address probe."""
    return bytes([
        req_id & 0xFF, (req_id >> 8) & 0xFF,  # id
        port, addr,
        0x00, 0x00,                           # interval_ms = 0
        0x01, 0x00,                           # count = 1
        0x01,                                 # read_len = 1
        0x00,                                 # write_len = 0
        0x00,                                 # post_len = 0
    ])


def probe_read(reader, ser, req_id, port, addr, timeout):
    """READ_REQUEST probe. Returns True if the device ACKed its address."""
    ser.reset_input_buffer()
    reader.buf.clear()
    ser.write(build_frame(CMD_READ_REQUEST, read_request_payload(req_id, port, addr)))
    ser.flush()
    deadline = time.monotonic() + timeout
    for cmd, payload in reader.poll(deadline):
        if cmd == CMD_READ_RESULT and len(payload) >= 5:
            rid = payload[0] | (payload[1] << 8)
            if rid == req_id:
                return payload[4] == ST_OK
    return None  # no response (firmware silent / wrong port?)


def probe_write(reader, ser, port, addr, timeout):
    """Zero-length I2C_WRITE probe. Reliable on PIO ports (2-7) only."""
    ser.reset_input_buffer()
    reader.buf.clear()
    ser.write(build_frame(CMD_I2C_WRITE, bytes([port, addr])))
    ser.flush()
    deadline = time.monotonic() + timeout
    for cmd, payload in reader.poll(deadline):
        if cmd == CMD_ACK and len(payload) >= 2 and payload[0] == CMD_I2C_WRITE:
            return payload[1] == ST_OK
    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def autodetect_port():
    cands = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return cands[0] if cands else None


def parse_int(s):
    return int(s, 0)  # honours 0x.. prefixes


def main():
    ap = argparse.ArgumentParser(description="I2C bus scanner for the ghost_sensor_host RP2040.")
    ap.add_argument("--port", help="serial device (default: autodetect /dev/ttyACM*)")
    ap.add_argument("--baud", type=int, default=115200, help="baud (nominal for USB CDC)")
    ap.add_argument("--ports", default="0-7",
                    help="host ports to scan, e.g. '6,7' or '0-7' (default all)")
    ap.add_argument("--start", type=parse_int, default=0x08, help="first I2C addr (default 0x08)")
    ap.add_argument("--end", type=parse_int, default=0x77, help="last I2C addr (default 0x77)")
    ap.add_argument("--timeout", type=float, default=0.1, help="per-probe timeout seconds")
    ap.add_argument("--write-probe", action="store_true",
                    help="probe with a zero-length write (PIO ports 2-7 only)")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if not port:
        sys.exit("error: no serial port found; pass --port /dev/ttyACMx")

    # Parse --ports into a sorted set.
    scan_ports = set()
    for tok in args.ports.split(","):
        tok = tok.strip()
        if "-" in tok:
            lo, hi = tok.split("-")
            scan_ports.update(range(int(lo), int(hi) + 1))
        elif tok:
            scan_ports.add(int(tok))
    scan_ports = sorted(p for p in scan_ports if 0 <= p < NUM_PORTS)
    if not scan_ports:
        sys.exit("error: no valid ports in --ports")

    if args.write_probe:
        print("note: --write-probe is unreliable on hardware ports 0,1 (Pico SDK "
              "skips zero-length writes); results there may be false.\n")

    try:
        ser = serial.Serial(port, args.baud, timeout=0)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {port}: {e}")

    print(f"Scanning {port} @ {args.baud} baud — addresses "
          f"0x{args.start:02x}..0x{args.end:02x}, "
          f"{'write' if args.write_probe else 'read'} probe\n")

    reader = FrameReader(ser)
    time.sleep(0.2)  # let the CDC link settle
    ser.reset_input_buffer()

    req_id = 0
    silent_ports = []
    total_found = 0

    for port_no in scan_ports:
        label, kind = PORT_INFO[port_no]
        found = []
        responded = False
        for addr in range(args.start, args.end + 1):
            if args.write_probe:
                present = probe_write(reader, ser, port_no, addr, args.timeout)
            else:
                req_id = (req_id + 1) & 0xFFFF
                present = probe_read(reader, ser, req_id, port_no, addr, args.timeout)
            if present is not None:
                responded = True
            if present:
                found.append(addr)

        if not responded:
            silent_ports.append(port_no)

        hits = ", ".join(
            f"0x{a:02x}" + (f" ({'; '.join(g)})" if (g := identify(a)) else "")
            for a in found
        ) if found else "—"
        print(f"port {port_no} (input {label[-1]}, {kind:6}): {hits}")
        total_found += len(found)

    ser.close()
    print(f"\n{total_found} device(s) found across {len(scan_ports)} port(s).")
    if silent_ports:
        print(f"warning: ports {silent_ports} never answered — is the host "
              f"flashed and on {port}?")


if __name__ == "__main__":
    main()
