#!/usr/bin/env python3
"""Exchange opaque test payloads through two V5 Brains and their VEXlink radio."""

import argparse
from contextlib import ExitStack
import os
import time

SYNC = b"\xAA\xBB"
HOST_TO_RADIO = 0x11
RADIO_TO_HOST = 0x12
IMU_DATA = 0x03
PAYLOAD_SIZE = 7
MAX_PAYLOAD_SIZE = 64
MAX_PENDING = 128
PENDING_TIMEOUT_S = 5.0
RADIO_PACKET_SIZE = 2 + PAYLOAD_SIZE


class SerialFrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                self.buffer[:] = SYNC[:1] if self.buffer.endswith(SYNC[:1]) else b""
                break
            if sync_index:
                del self.buffer[:sync_index]
            if len(self.buffer) < 4:
                break

            payload_size = self.buffer[3]
            if payload_size > MAX_PAYLOAD_SIZE:
                del self.buffer[0]
                continue
            packet_size = 4 + payload_size
            if len(self.buffer) < packet_size:
                break

            msg_type = self.buffer[2]
            payload = bytes(self.buffer[4:packet_size])
            del self.buffer[:packet_size]
            frames.append((msg_type, payload))
        return frames


def encode_host_frame(payload):
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError(f"Payload must contain exactly {PAYLOAD_SIZE} bytes")
    return SYNC + bytes((HOST_TO_RADIO, PAYLOAD_SIZE)) + payload


def run_self_tests():
    payload = bytes((0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0xFF))
    encoded = encode_host_frame(payload)
    assert encoded == SYNC + b"\x11\x07" + payload

    parser = SerialFrameParser()
    assert parser.feed(b"noise\xAA") == []
    assert parser.feed(encoded[1:5]) == []
    assert parser.feed(encoded[5:]) == [(HOST_TO_RADIO, payload)]
    radio_output = SYNC + bytes((RADIO_TO_HOST, PAYLOAD_SIZE)) + payload
    assert parser.feed(radio_output) == [(RADIO_TO_HOST, payload)]
    assert parser.feed(SYNC + bytes((IMU_DATA, 2)) + b"\x01\x02") == [
        (IMU_DATA, b"\x01\x02")
    ]

    parser = SerialFrameParser()
    malformed = SYNC + bytes((RADIO_TO_HOST, MAX_PAYLOAD_SIZE + 1))
    assert parser.feed(malformed + encoded) == [(HOST_TO_RADIO, payload)]
    print("Host serial framing self-test passed")


def run_client(args):
    try:
        import serial
    except ImportError as error:
        raise SystemExit("Install pyserial to use the host client: python3 -m pip install pyserial") from error

    endpoint_ports = {
        "manager": args.manager_port,
        "worker": args.worker_port,
    }
    parsers = {name: SerialFrameParser() for name in endpoint_ports}
    sent = {"manager": 0, "worker": 0}
    received = {"manager": 0, "worker": 0}
    timed_out = 0
    unmatched = 0
    pending = {}
    start_at = time.monotonic()
    next_send = start_at
    stop_at = next_send + args.duration_s if args.duration_s else None
    interval_s = args.interval_ms / 1000.0

    with ExitStack() as stack:
        ports = {
            name: stack.enter_context(
                serial.Serial(port, baudrate=args.baud, timeout=0, write_timeout=1)
            )
            for name, port in endpoint_ports.items()
        }
        try:
            while stop_at is None or time.monotonic() < stop_at:
                now = time.monotonic()
                for key, sent_at in list(pending.items()):
                    if now - sent_at[1] > PENDING_TIMEOUT_S:
                        del pending[key]
                        timed_out += 1

                if now >= next_send and len(pending) + 2 <= MAX_PENDING:
                    for sender in ("manager", "worker"):
                        destination = "worker" if sender == "manager" else "manager"
                        payload = os.urandom(PAYLOAD_SIZE)
                        while (destination, payload) in pending:
                            payload = os.urandom(PAYLOAD_SIZE)
                        ports[sender].write(encode_host_frame(payload))
                        pending[(destination, payload)] = (sender, now)
                        sent[sender] += 1
                    next_send = now + interval_s

                for destination, port in ports.items():
                    data = port.read(256)
                    for msg_type, payload in parsers[destination].feed(data):
                        if msg_type == IMU_DATA:
                            continue
                        if msg_type != RADIO_TO_HOST or len(payload) != PAYLOAD_SIZE:
                            continue

                        record = pending.pop((destination, payload), None)
                        if record is None:
                            unmatched += 1
                            print(f"{destination} received opaque payload {payload.hex()} (unmatched)")
                            continue
                        sender, sent_at = record
                        received[destination] += 1
                        latency_ms = (time.monotonic() - sent_at) * 1000.0
                        print(
                            f"{sender} -> {destination}: {payload.hex()} "
                            f"({latency_ms:.1f} ms)"
                        )

                time.sleep(0.005)
        except KeyboardInterrupt:
            pass

    elapsed_s = max(time.monotonic() - start_at, 0.001)
    print(
        f"Elapsed: {elapsed_s:.1f} s; timed out: {timed_out}; "
        f"still pending: {len(pending)}; unmatched: {unmatched}"
    )
    print(
        f"Manager -> worker: {received['worker']}/{sent['manager']} packets, "
        f"{received['worker'] * RADIO_PACKET_SIZE / elapsed_s:.1f} B/s"
    )
    print(
        f"Worker -> manager: {received['manager']}/{sent['worker']} packets, "
        f"{received['manager'] * RADIO_PACKET_SIZE / elapsed_s:.1f} B/s"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="test serial framing without hardware")
    parser.add_argument("--manager-port", help="USB serial device connected to the manager Brain")
    parser.add_argument("--worker-port", help="USB serial device connected to the worker Brain")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval-ms", type=int, default=100)
    parser.add_argument("--duration-s", type=float, default=0, help="run for this long; 0 runs until interrupted")
    args = parser.parse_args()

    if args.self_test:
        run_self_tests()
        return
    if not args.manager_port or not args.worker_port:
        parser.error("--manager-port and --worker-port are required unless --self-test is used")
    if args.interval_ms < 80:
        parser.error("--interval-ms must be at least 80 to stay below the worker send cadence")
    if args.baud <= 0 or args.duration_s < 0:
        parser.error("--baud must be positive and --duration-s cannot be negative")

    run_client(args)


if __name__ == "__main__":
    main()
