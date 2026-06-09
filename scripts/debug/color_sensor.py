#!/usr/bin/env python3
"""Live colour swatch for the sensor host colour sensor, in the terminal.

Renders the published RGB as a 24-bit truecolor block (works over SSH, no GUI).

Usage:
    python3 color_sensor.py [topic]

`topic` defaults to /sensor_host/color_sensor_update.
Ctrl-C to quit. Requires a truecolor-capable terminal.
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ghost_msgs.msg import ColorSensorState

DEFAULT_TOPIC = "/sensor_host/color_sensor_update"


def render(name, r16, g16, b16):
    # 16-bit sensor channels -> 8-bit display.
    r, g, b = r16 >> 8, g16 >> 8, b16 >> 8
    bg = f"\x1b[48;2;{r};{g};{b}m"
    out = ["\x1b[H"]  # cursor home
    for _ in range(8):
        out.append(bg + " " * 44 + "\x1b[0m\x1b[K\n")
    out.append(f"\x1b[K {name}\n")
    out.append(f"\x1b[K R:{r16:5d}  G:{g16:5d}  B:{b16:5d}   (8-bit {r:3d},{g:3d},{b:3d})\n")
    out.append("\x1b[K Ctrl-C to quit\n")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC

    rclpy.init()
    node = Node("color_sensor_debug")
    node.create_subscription(
        ColorSensorState, topic,
        lambda m: render(m.name, m.r, m.g, m.b),
        qos_profile_sensor_data)

    sys.stdout.write("\x1b[2J\x1b[?25l")  # clear screen, hide cursor
    sys.stdout.flush()
    print(f"Listening on {topic} ...")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        sys.stdout.write("\x1b[?25h\n")  # restore cursor
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
